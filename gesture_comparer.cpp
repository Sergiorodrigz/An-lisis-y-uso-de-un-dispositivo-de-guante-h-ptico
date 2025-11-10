// gesture_comparer.cpp
// - record: guarda gesto (mediana de 5 muestras)
// - list:   lista nombres únicos desde gestures.csv
// - try:    intenta reproducir un gesto y verifica Correcto/Incorrecto
// - threshold: ver/cambiar umbral de similitud

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
#include <map>
#include <algorithm>
#include <cctype>
#include <limits>

using namespace SGCore;

static const char* kOutputCSV = "gestures.csv";

// ---------- utilidades de string ----------

static inline std::string trim(const std::string& s) {
    size_t a = 0, b = s.size();
    while (a < b && std::isspace((unsigned char)s[a])) ++a;
    while (b > a && std::isspace((unsigned char)s[b-1])) --b;
    return s.substr(a, b-a);
}

// Levenshtein iterativo (memoria O(m)) y normalizado por max(lenA,lenB)
static double normalizedLevenshtein(const std::string& a, const std::string& b) {
    const size_t n = a.size(), m = b.size();
    if (n == 0 && m == 0) return 0.0;
    std::vector<size_t> dp(m + 1);
    for (size_t j = 0; j <= m; ++j) dp[j] = j;
    for (size_t i = 1; i <= n; ++i) {
        size_t prev = dp[0]; dp[0] = i;
        for (size_t j = 1; j <= m; ++j) {
            size_t tmp = dp[j];
            if (a[i - 1] == b[j - 1]) dp[j] = prev;
            else dp[j] = std::min({ prev, dp[j], dp[j - 1] }) + 1;
            prev = tmp;
        }
    }
    double dist = (double)dp[m];
    double norm = (double)std::max(n, m);
    return dist / norm; // 0..1
}

// ---------- CSV: leer/escribir ----------

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
            std::string name = trim(line.substr(0, p));
            if (!name.empty()) uniq.insert(name);
        }
    }
    return std::vector<std::string>(uniq.begin(), uniq.end());
}

std::map<std::string, std::vector<std::string>> LoadGestureMap(const std::string& csvPath)
{
    std::map<std::string, std::vector<std::string>> M;
    std::ifstream ifs(csvPath);
    if (!ifs) return M;
    std::string line;
    while (std::getline(ifs, line))
    {
        auto p = line.find(';');
        if (p == std::string::npos) continue;
        std::string name = trim(line.substr(0, p));
        std::string pose = trim(line.substr(p + 1));
        if (!name.empty() && !pose.empty()) M[name].push_back(pose);
    }
    return M;
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

// ---------- Inicialización SenseGlove ----------

bool InitializeSystem()
{
    if (!SenseCom::ScanningActive())
    {
        std::cout << "SenseCom no está activo; intentando iniciarlo...\n";
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

bool CaptureHandPoseOnce(bool rightHand, SGCore::HandPose& pose)
{
    for (int r = 0; r < 5; ++r)
    {
        if (HandLayer::GetHandPose(rightHand, pose)) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return false;
}

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

    size_t bestIdx = 0;
    double bestCost = std::numeric_limits<double>::infinity();
    for (int i = 0; i < samples; ++i)
    {
        double cost = 0.0;
        for (int j = 0; j < samples; ++j) if (i != j) cost += normalizedLevenshtein(asText[i], asText[j]);
        if (cost < bestCost) { bestCost = cost; bestIdx = (size_t)i; }
    }
    outPose = poses[bestIdx];
    return true;
}

// ---------- Comparación ----------

struct CompareResult {
    double bestSimilarity = 0.0;   // 0..1 (1 = idéntico)
    std::string matchedSample;     // la muestra en CSV que mejor encajó
};

CompareResult CompareWithGestureSamples(
    const std::vector<std::string>& samples, // poses guardadas (texto)
    const std::string& currentPoseStr
){
    CompareResult R;
    if (samples.empty()) return R;
    for (auto& saved : samples)
    {
        double d = normalizedLevenshtein(saved, currentPoseStr); // 0..1
        double sim = 1.0 - d; // 1 = muy parecido
        if (sim > R.bestSimilarity) {
            R.bestSimilarity = sim;
            R.matchedSample = saved;
        }
    }
    return R;
}

// ---------- Main ----------

int main()
{
    std::cout << "SenseGlove Gesture Comparer (median-of-5 + list + try)\n";

    if (!InitializeSystem()) return 1;

    bool useRightHand = true;
    if (HandLayer::GlovesConnected() == 1) useRightHand = HandLayer::GetFirstGloveHandedness();
    else useRightHand = HandLayer::DeviceConnected(true) ? true : false;
    std::cout << "Usando mano " << (useRightHand ? "derecha" : "izquierda") << ".\n";
    std::cout << "Dataset: \"" << kOutputCSV << "\"\n";

    double similarityThreshold = 0.88; // ajustable
    std::cout << "Umbral de similitud inicial: " << similarityThreshold << " (1.0=idéntico)\n";

    std::cout << "\nComandos:\n"
              << "  record         -> registrar un nuevo gesto\n"
              << "  list           -> listar gestos ya guardados\n"
              << "  try            -> elegir gesto y comprobar si coincide (Correcto/Incorrecto)\n"
              << "  threshold [v]  -> ver/cambiar umbral (0..1)\n"
              << "  help           -> mostrar ayuda\n"
              << "  exit           -> salir\n";

    std::string cmd;
    while (true)
    {
        std::cout << "\n> ";
        if (!std::getline(std::cin, cmd)) break;
        cmd = trim(cmd);
        if (cmd.empty()) continue;
        if (cmd == "exit" || cmd == "quit") break;

        if (cmd == "help")
        {
            std::cout << "record: captura mediana de 5 muestras y guarda en CSV\n"
                      << "list:   lista nombres únicos presentes en " << kOutputCSV << "\n"
                      << "try:    selecciona gesto y compara contra tu pose (Correcto/Incorrecto)\n"
                      << "threshold [v]: muestra o fija el umbral (ej. threshold 0.90)\n";
            continue;
        }

        if (cmd.rfind("threshold", 0) == 0)
        {
            std::string rest = trim(cmd.substr(std::string("threshold").size()));
            if (rest.empty()) {
                std::cout << "Umbral actual = " << similarityThreshold << "\n";
            } else {
                try {
                    double v = std::stod(rest);
                    if (v > 0.0 && v <= 1.0) { similarityThreshold = v; std::cout << "Nuevo umbral = " << similarityThreshold << "\n"; }
                    else std::cout << "Valor inválido. Usa (0, 1].\n";
                } catch (...) { std::cout << "Formato inválido. Ej: threshold 0.9\n"; }
            }
            continue;
        }

        if (cmd == "list")
        {
            auto names = ListGestureNames(kOutputCSV);
            if (names.empty()) { std::cout << "(no hay gestos guardados aún)\n"; continue; }
            std::cout << "Gestos:\n";
            for (auto& n : names) std::cout << " - " << n << "\n";
            continue;
        }

        if (cmd == "record")
        {
            std::cout << "Nombre del gesto: ";
            std::string gestureName;
            if (!std::getline(std::cin, gestureName) || trim(gestureName).empty()) { std::cout << "Cancelado.\n"; continue; }
            gestureName = trim(gestureName);

            std::cout << "Mantén la postura... (5 muestras ~250 ms)\n";
            HandPose pose;
            if (!CaptureMedianPose(useRightHand, pose, 5, 50))
            {
                std::cout << "No se pudo capturar la pose. Verifica conexión y calibración.\n";
                continue;
            }
            SaveGesture(kOutputCSV, gestureName, pose.ToString());
            std::cout << "Gesto '" << gestureName << "' guardado.\n";
            continue;
        }

        if (cmd == "try")
        {
            auto names = ListGestureNames(kOutputCSV);
            if (names.empty()) { std::cout << "(no hay gestos guardados aún)\n"; continue; }

            std::cout << "Elige un gesto:\n";
            for (size_t i=0;i<names.size();++i) std::cout << "  [" << i << "] " << names[i] << "\n";
            std::cout << "Índice: ";
            std::string sidx;
            if (!std::getline(std::cin, sidx)) continue;
            int idx = -1;
            try { idx = std::stoi(sidx); } catch (...) { idx = -1; }
            if (idx < 0 || idx >= (int)names.size()) { std::cout << "Índice inválido.\n"; continue; }

            std::string chosen = names[(size_t)idx];
            auto G = LoadGestureMap(kOutputCSV);
            auto it = G.find(chosen);
            if (it == G.end() || it->second.empty()) { std::cout << "No hay muestras para '"<<chosen<<"'.\n"; continue; }

            std::cout << "Coloca la mano en el gesto '" << chosen << "'. Capturando (5 muestras)...\n";
            HandPose pose;
            if (!CaptureMedianPose(useRightHand, pose, 5, 50))
            {
                std::cout << "No se pudo capturar la pose.\n";
                continue;
            }
            std::string current = pose.ToString();

            CompareResult R = CompareWithGestureSamples(it->second, current);
            double sim = R.bestSimilarity; // 0..1
            std::cout << "Similitud: " << sim << "  (umbral=" << similarityThreshold << ")\n";
            if (sim >= similarityThreshold) {
                std::cout << "Resultado: **Correcto** \n";
            } else {
                std::cout << "Resultado: **Incorrecto** \n";
            }
            continue;
        }

        // atajo: si teclean un nombre cualquiera, registra ese gesto
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
