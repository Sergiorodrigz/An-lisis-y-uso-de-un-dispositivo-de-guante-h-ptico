// gesture_recorder.cpp  

#include <SenseGlove/Connect/SGConnect.hpp>
#include <SenseGlove/Core/Library.hpp>
#include <SenseGlove/Core/SenseCom.hpp>
#include <SenseGlove/Core/HandLayer.hpp>
#include <SenseGlove/Core/HandPose.hpp>

#include <chrono>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include <set>
#include <algorithm>

using namespace SGCore;



static const char* kOutputCSV = "gestures.csv";


std::vector<std::string> ListGestureNames(const std::string& csvPath)
{
    std::ifstream ifs(csvPath);
    std::set<std::string> uniq;
    if (!ifs) return {};

    std::string line;
    while (std::getline(ifs, line))
    {
        auto p = line.find(';');
        if (p != std::string::npos)
        {
            std::string name = line.substr(0, p);
            
            while (!name.empty() && (name.back()==' ' || name.back()=='\t')) name.pop_back();
            while (!name.empty() && (name.front()==' ' || name.front()=='\t')) name.erase(name.begin());
            if (!name.empty()) uniq.insert(name);
        }
    }
    return std::vector<std::string>(uniq.begin(), uniq.end());
}

void SaveGesture(const std::string& filename, const std::string& gestureName, const std::string& poseString)
{
    std::ofstream ofs(filename, std::ios::app);
    if (!ofs)
    {
        std::cout << "Error: no se pudo abrir " << filename << " para escritura.\n";
        return;
    }
    std::string cleaned = poseString;
    for (char& c : cleaned) if (c == '\n' || c == '\r') c = ' ';
    ofs << gestureName << ";" << cleaned << "\n";
}

// ---------- Inicialización ----------

bool InitializeSystem()
{
    if (!SenseCom::ScanningActive())
    {
        std::cout << "SenseCom no esta activo; intentando iniciarlo...\n";
        if (!SenseCom::StartupSenseCom())
        {
            std::cout << "Fallo al iniciar SenseCom. Asegúrate de haberlo instalado y ejecutado al menos una vez.\n";
            return false;
        }
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }
    int attempts = 0;
    while (HandLayer::GlovesConnected() == 0 && attempts < 10)
    {
        std::cout << "Esperando guante SenseGlove...\n";
        std::this_thread::sleep_for(std::chrono::seconds(1));
        ++attempts;
    }
    if (HandLayer::GlovesConnected() == 0)
    {
        std::cout << "No se detectaron guantes hápticos.\n";
        return false;
    }
    return true;
}

// ---------- Captura ----------

// Captura una única HandPose con reintentos breves
bool CaptureHandPoseOnce(bool rightHand, SGCore::HandPose& pose)
{
    for (int r = 0; r < 5; ++r)
    {
        if (HandLayer::GetHandPose(rightHand, pose)) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return false;
}

// Captura N muestras y devuelve la "mediana" (por estabilidad temporal)

bool CaptureMedianPose(bool rightHand, SGCore::HandPose& outPose, int samples = 5, int intervalMs = 50)
{
    if (samples <= 1) return CaptureHandPoseOnce(rightHand, outPose);

    std::vector<std::string> asText; asText.reserve(samples);
    std::vector<SGCore::HandPose> poses; poses.reserve(samples);

    SGCore::HandPose p;
    for (int i = 0; i < samples; ++i)
    {
        if (!CaptureHandPoseOnce(rightHand, p)) return false;
        poses.push_back(p);
        asText.push_back(p.ToString());
        std::this_thread::sleep_for(std::chrono::milliseconds(intervalMs));
    }

    // Elegimos la muestra cuyo texto es "más similar" al resto.
    // (A falta de acceso numérico a articulaciones, usamos distancia de Levenshtein simple)
    auto dist = [](const std::string& a, const std::string& b) {
        // Levenshtein iterativo (optimizado en memoria)
        const size_t n = a.size(), m = b.size();
        std::vector<size_t> dp(m+1);
        for (size_t j=0;j<=m;++j) dp[j]=j;
        for (size_t i=1;i<=n;++i){
            size_t prev=dp[0]; dp[0]=i;
            for (size_t j=1;j<=m;++j){
                size_t tmp=dp[j];
                if (a[i-1]==b[j-1]) dp[j]=prev;
                else dp[j]=std::min({prev, dp[j], dp[j-1]})+1;
                prev=tmp;
            }
        }
        return dp[m];
    };

    size_t bestIdx = 0;
    size_t bestCost = SIZE_MAX;
    for (int i = 0; i < samples; ++i)
    {
        size_t cost = 0;
        for (int j = 0; j < samples; ++j) if (i != j) cost += dist(asText[i], asText[j]);
        if (cost < bestCost) { bestCost = cost; bestIdx = (size_t)i; }
    }
    outPose = poses[bestIdx];
    return true;
}

// ---------- Main ----------

int main()
{
    std::cout << "SenseGlove Gesture Recorder (media de 5 muestras + listado)\n";

    if (!InitializeSystem()) return 1;

    bool useRightHand = true;
    if (HandLayer::GlovesConnected() == 1) useRightHand = HandLayer::GetFirstGloveHandedness();
    else useRightHand = HandLayer::DeviceConnected(true) ? true : false;
    std::cout << "Usando mano " << (useRightHand ? "derecha" : "izquierda") << ".\n";
    std::cout << "Guardando en \"" << kOutputCSV << "\"\n";

    std::cout << "\nComandos:\n"
              << "  record   -> registrar un nuevo gesto\n"
              << "  list     -> listar gestos ya guardados\n"
              << "  help     -> mostrar ayuda\n"
              << "  exit     -> salir\n";

    std::string cmd;
    while (true)
    {
        std::cout << "\n> ";
        if (!std::getline(std::cin, cmd)) break;

        // trim
        while (!cmd.empty() && (cmd.back()==' '||cmd.back()=='\t')) cmd.pop_back();
        while (!cmd.empty() && (cmd.front()==' '||cmd.front()=='\t')) cmd.erase(cmd.begin());
        if (cmd.empty()) continue;

        if (cmd == "exit" || cmd == "quit") break;

        if (cmd == "help")
        {
            std::cout << "record: pide nombre y captura mediana de 5 muestras\n"
                      << "list: muestra nombres unicos presentes en " << kOutputCSV << "\n";
            continue;
        }

        if (cmd == "list")
        {
            auto names = ListGestureNames(kOutputCSV);
            if (names.empty()) { std::cout << "(no hay gestos guardados aún)\n"; continue; }
            std::cout << "Gestos en " << kOutputCSV << ":\n";
            for (auto& n : names) std::cout << " - " << n << "\n";
            continue;
        }

        if (cmd == "record")
        {
            std::cout << "Nombre del gesto: ";
            std::string gestureName;
            if (!std::getline(std::cin, gestureName) || gestureName.empty()) { std::cout << "Cancelado.\n"; continue; }

            std::cout << "Manten la postura... (tomando 5 muestras ~250 ms)\n";
            HandPose pose;
            if (!CaptureMedianPose(useRightHand, pose, /*samples*/5, /*intervalMs*/50))
            {
                std::cout << "No se pudo capturar la pose. Verifica conexion y calibracion.\n";
                continue;
            }
            std::string poseStr = pose.ToString();
            SaveGesture(kOutputCSV, gestureName, poseStr);
            std::cout << "Gesto '" << gestureName << "' guardado.\n";
            continue;
        }

        // Si el usuario teclea directamente un nombre, lo interpretamos como "record ese nombre"
        if (!cmd.empty())
        {
            std::string gestureName = cmd;
            std::cout << "Mantén la postura para '" << gestureName << "'... (5 muestras)\n";
            HandPose pose;
            if (!CaptureMedianPose(useRightHand, pose, 5, 50))
            {
                std::cout << "No se pudo capturar la pose.\n";
                continue;
            }
            SaveGesture(kOutputCSV, gestureName, pose.ToString());
            std::cout << "Gesto '" << gestureName << "' guardado.\n";
        }
    }

    std::cout << "Saliendo.\n";
    return 0;
}
