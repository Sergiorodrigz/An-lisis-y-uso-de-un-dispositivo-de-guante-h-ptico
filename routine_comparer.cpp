// routine_comparer.cpp - SenseGlove: Rutinas por gestos con fases de fuerza
//
// Cambios clave:
//  - CSV con dificultad: "gesture_name;difficulty;pose_string"
//    (compatible con formato antiguo "gesture_name;pose_string")
//  - Comando routine: recorre 3 gestos (de menor dificultad media a mayor) y
//    ejecuta la rutina completa a 0%, luego 25% y luego 50% de fuerza.
//  - Telemetria y log: resumen por consola y export a CSV (routine_log.csv)
//  - Sin tildes ni simbolos griegos en comentarios y cadenas.
//
// Compilacion tipica (depende de tu entorno SGCore):
//   MSVC: cl /EHsc /std:c++17 /MD routine_comparer.cpp /I"<SGCORE>/include" ^
//         /link /LIBPATH:"<SGCORE>/lib/win64/msvc143/release" sgcore.lib sgconnect.lib Ws2_32.lib
//
// -----------------------------------------------------------------------------
// CAPAS (en un solo archivo para simplicidad):
//   Persistencia CSV
//   Adquisicion (SDK)
//   Normalizacion (pose->features)
//   Plantillas y comparador
//   Haptics
//   CLI + Rutina + Telemetria/Log
// -----------------------------------------------------------------------------

#include <SenseGlove/Connect/SGConnect.hpp>
#include <SenseGlove/Core/Library.hpp>
#include <SenseGlove/Core/SenseCom.hpp>
#include <SenseGlove/Core/HandLayer.hpp>
#include <SenseGlove/Core/HandPose.hpp>
#include <SenseGlove/Core/HapticGlove.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <regex>
#include <set>
#include <string>
#include <thread>
#include <vector>

using namespace SGCore;

// =============================================================================
// Persistencia (CSV)
// =============================================================================

static const char* kOutputCSV = "gestures.csv";

static inline std::string trim(const std::string& s){
    size_t a=0,b=s.size();
    while(a<b && std::isspace((unsigned char)s[a])) ++a;
    while(b>a && std::isspace((unsigned char)s[b-1])) --b;
    return s.substr(a,b-a);
}

struct Entry {
    std::string name;
    int         difficulty = 1; // entero >=1
    std::string pose;
};

std::vector<Entry> LoadEntries(const std::string& csvPath){
    std::vector<Entry> out;
    std::ifstream ifs(csvPath);
    if(!ifs) return out;
    std::string line;
    while(std::getline(ifs,line)){
        auto p = line.find(';');
        if (p==std::string::npos) continue;
        std::string name = trim(line.substr(0,p));
        std::string rest = trim(line.substr(p+1));
        if (name.empty() || rest.empty()) continue;

        // Formato nuevo: name;difficulty;pose
        // Formato antiguo: name;pose
        Entry e; e.name = name;
        auto q = rest.find(';');
        if (q==std::string::npos){
            // antiguo
            e.difficulty = 1;
            e.pose = rest;
        } else {
            std::string diffStr = trim(rest.substr(0,q));
            e.pose = trim(rest.substr(q+1));
            try { e.difficulty = std::max(1, std::stoi(diffStr)); }
            catch(...) { e.difficulty = 1; }
        }
        if (!e.pose.empty()) out.push_back(e);
    }
    return out;
}

void SaveGesture(const std::string& filename,
                 const std::string& gestureName,
                 int difficulty,
                 const std::string& poseString)
{
    std::ofstream ofs(filename, std::ios::app);
    if (!ofs){ std::cout<<"Error abriendo "<<filename<<" para escritura.\n"; return; }
    std::string cleaned = poseString;
    for(char& c: cleaned) if(c=='\n'||c=='\r') c=' ';
    ofs << gestureName << ";" << difficulty << ";" << cleaned << "\n";
}

std::vector<std::string> ListGestureNames(const std::string& csvPath){
    auto E = LoadEntries(csvPath);
    std::set<std::string> S;
    for (auto& e:E) if(!e.name.empty()) S.insert(e.name);
    return std::vector<std::string>(S.begin(), S.end());
}

// =============================================================================
// Adquisicion (SDK)
// =============================================================================

bool InitializeSystem(){
    if (!SenseCom::ScanningActive()){
        std::cout<<"SenseCom no activo; intentando iniciarlo...\n";
        if (!SenseCom::StartupSenseCom()){
            std::cout<<"Fallo al iniciar SenseCom.\n";
            return false;
        }
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }
    for(int t=0; t<10 && HandLayer::GlovesConnected()==0; ++t){
        std::cout<<"Esperando guante...\n";
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    if (HandLayer::GlovesConnected()==0){
        std::cout<<"No se detectaron guantes.\n";
        return false;
    }
    return true;
}

bool CaptureHandPoseOnce(bool rightHand, SGCore::HandPose& pose){
    for(int r=0;r<5;++r){
        if (HandLayer::GetHandPose(rightHand, pose)) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return false;
}

bool CaptureMedianPose(bool rightHand, SGCore::HandPose& outPose, int samples=5, int intervalMs=50){
    if(samples<=1) return CaptureHandPoseOnce(rightHand, outPose);

    std::vector<std::string> asText; asText.reserve(samples);
    std::vector<SGCore::HandPose> poses; poses.reserve(samples);

    SGCore::HandPose p;
    for(int i=0;i<samples;++i){
        if(!CaptureHandPoseOnce(rightHand,p)) return false;
        poses.push_back(p);
        asText.push_back(p.ToString());
        std::this_thread::sleep_for(std::chrono::milliseconds(intervalMs));
    }

    auto levNorm=[&](const std::string& a, const std::string& b){
        const size_t n=a.size(), m=b.size();
        if(n==0 && m==0) return 0.0;
        std::vector<size_t> dp(m+1);
        for(size_t j=0;j<=m;++j) dp[j]=j;
        for(size_t i=1;i<=n;++i){
            size_t prev=dp[0]; dp[0]=i;
            for(size_t j=1;j<=m;++j){
                size_t tmp=dp[j];
                dp[j] = (a[i-1]==b[j-1]) ? prev : std::min({prev,dp[j],dp[j-1]})+1;
                prev = tmp;
            }
        }
        return double(dp[m]) / double(std::max(n,m));
    };

    size_t bestIdx=0; double bestCost=std::numeric_limits<double>::infinity();
    for(int i=0;i<samples;++i){
        double cost=0.0;
        for(int j=0;j<samples;++j) if(i!=j) cost+=levNorm(asText[i],asText[j]);
        if(cost<bestCost){ bestCost=cost; bestIdx=(size_t)i; }
    }
    outPose = poses[bestIdx];
    return true;
}

// =============================================================================
// Normalizacion (parseo de pose -> 21 features)
// =============================================================================

static const std::vector<std::string> kOrder = {
    // Thumb (5)
    "Thumb.CMC-Abd","Thumb.CMC-Tw","Thumb.CMC-Flx","Thumb.MCP-Flx","Thumb.IP-Flx",
    // Index (4)
    "Index.MCP-Abd","Index.MCP-Flx","Index.PIP-Flx","Index.DIP-Flx",
    // Middle (4)
    "Middle.MCP-Abd","Middle.MCP-Flx","Middle.PIP-Flx","Middle.DIP-Flx",
    // Ring (4)
    "Ring.MCP-Abd","Ring.MCP-Flx","Ring.PIP-Flx","Ring.DIP-Flx",
    // Pinky (4)
    "Pinky.MCP-Abd","Pinky.MCP-Flx","Pinky.PIP-Flx","Pinky.DIP-Flx"
};

bool ExtractFeaturesFromPoseString(const std::string& s, std::vector<double>& out){
    out.assign(kOrder.size(), std::numeric_limits<double>::quiet_NaN());
    std::string t=s;

    auto findVal = [&](const std::string& finger, const std::string& key, double& val)->bool{
        try{
            std::regex rg(finger + ".*?" + key + R"(\s*:\s*(-?\d+(\.\d+)?))", std::regex::icase);
            std::smatch m;
            if (std::regex_search(t,m,rg)){ val=std::stod(m[1].str()); return true; }
        } catch(...) {}
        return false;
    };
    auto set=[&](const std::string& dotKey,double v){
        auto it=std::find(kOrder.begin(),kOrder.end(),dotKey);
        if(it!=kOrder.end()) out[size_t(it-kOrder.begin())]=v;
    };

    { double v;
      if(findVal("Thumb","CMC-Abd",v)) set("Thumb.CMC-Abd",v);
      if(findVal("Thumb","CMC-Tw", v)) set("Thumb.CMC-Tw", v);
      if(findVal("Thumb","CMC-Flx",v)) set("Thumb.CMC-Flx",v);
      if(findVal("Thumb","MCP-Flx",v)) set("Thumb.MCP-Flx",v);
      if(findVal("Thumb","IP-Flx", v)) set("Thumb.IP-Flx", v);
    }
    for (auto f : {"Index","Middle","Ring","Pinky"}){
        double v;
        if(findVal(f,"MCP-Abd",v)) set(std::string(f)+".MCP-Abd",v);
        if(findVal(f,"MCP-Flx",v)) set(std::string(f)+".MCP-Flx",v);
        if(findVal(f,"PIP-Flx",v)) set(std::string(f)+".PIP-Flx",v);
        if(findVal(f,"DIP-Flx",v)) set(std::string(f)+".DIP-Flx",v);
    }

    size_t ok=0; for(double v:out) if(!std::isnan(v)) ++ok;
    return ok >= (out.size()*7)/10; // al menos 70 por ciento presentes
}

// =============================================================================
// Plantillas y comparador (mean, stdev)
// =============================================================================

struct TemplateStats {
    std::vector<double> mean;
    std::vector<double> stdev;
    size_t n=0;
};

bool BuildGestureTemplate(const std::vector<Entry>& all,
                          const std::string& gesture,
                          TemplateStats& T)
{
    const size_t F = kOrder.size();
    std::vector<double> sum(F,0.0), sum2(F,0.0);
    std::vector<size_t> count(F,0);

    size_t used=0;
    std::vector<double> feat;
    for (auto& e: all){
        if (e.name!=gesture) continue;
        if (!ExtractFeaturesFromPoseString(e.pose, feat)) continue;
        ++used;
        for(size_t i=0;i<F;++i){
            double v = feat[i];
            if (!std::isnan(v)){ sum[i]+=v; sum2[i]+=v*v; count[i]++; }
        }
    }
    if(used==0) return false;

    T.mean.assign(F, std::numeric_limits<double>::quiet_NaN());
    T.stdev.assign(F, std::numeric_limits<double>::quiet_NaN());
    T.n = used;

    for(size_t i=0;i<F;++i){
        if(count[i]==0) continue;
        double m = sum[i]/double(count[i]);
        double var = std::max(0.0, sum2[i]/double(count[i]) - m*m);
        T.mean[i]=m; T.stdev[i]=std::sqrt(var);
    }
    return true;
}

// score en [0,1]. k escala la tolerancia por componente; eps en grados.
double ScoreToTemplate(const std::vector<double>& x,
                       const TemplateStats& T,
                       double k=1.0, double eps=3.0)
{
    const size_t F=kOrder.size();
    double num=0.0, den=0.0; size_t used=0;
    for(size_t i=0;i<F;++i){
        double m=T.mean[i], s=T.stdev[i], xi=x[i];
        if(std::isnan(m)||std::isnan(xi)) continue;
        double tol=std::max(k*s, eps);
        double err=std::abs(xi-m);
        num += std::min(err,tol);
        den += tol;
        ++used;
    }
    if(used==0 || den<=0) return 0.0;
    double score = 1.0 - (num/den);
    if(score<0) score=0; if(score>1) score=1;
    return score;
}

// =============================================================================
// Haptics / Control de fuerza
// =============================================================================

void ApplyForce(bool rightHand, float level /*0..1*/){
    std::vector<float> ffb = { level, level, level, level, level }; // 5 dedos
    HandLayer::QueueCommand_ForceFeedbackLevels(rightHand, ffb, true);
}
void StopForce(bool rightHand){
    ApplyForce(rightHand, 0.0f);
}

// =============================================================================
// Telemetria y Log
// =============================================================================

static inline std::string NowISO8601() {
    using namespace std::chrono;
    auto now = system_clock::now();
    std::time_t t = system_clock::to_time_t(now);
    std::tm tm{};
#ifdef _WIN32
    localtime_s(&tm, &t);
#else
    localtime_r(&t, &tm);
#endif
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S", &tm);
    return std::string(buf);
}

struct RoutineResult {
    std::string timestamp;    // ISO-8601
    std::string routineId;    // mismo id para toda la ejecucion
    std::string gesture;      // nombre del gesto
    int  phase = 0;           // 1..N
    float forcePct = 0.f;     // 0..1 (se guarda 0..100 en CSV)
    bool success = false;     // superado o no
    int  attempts = 0;        // numero de intentos
    double bestScore = 0.0;   // mejor score visto
    int  timeToSuccessMs = -1;// ms hasta el primer exito (o -1)
    double threshold = 0.0;   // umbral
    double kTol = 0.0;        // k de tolerancia
};

static inline void AppendRoutineLogCSV(const std::string& path,
                                       const std::vector<RoutineResult>& rows)
{
    bool exists = std::filesystem::exists(path);
    std::ofstream f(path, std::ios::app);
    if (!f) { std::cout << "No se pudo abrir log " << path << "\n"; return; }
    if (!exists) {
        f << "timestamp,routine_id,gesture,phase,force_pct,success,attempts,"
             "best_score,time_to_success_ms,threshold,kTol\n";
    }
    for (auto& r : rows) {
        f << r.timestamp << ","
          << r.routineId << ","
          << r.gesture << ","
          << r.phase << ","
          << int(r.forcePct * 100) << ","
          << (r.success ? 1 : 0) << ","
          << r.attempts << ","
          << r.bestScore << ","
          << r.timeToSuccessMs << ","
          << r.threshold << ","
          << r.kTol << "\n";
    }
}

static inline void PrintRoutineSummary(const std::vector<RoutineResult>& rows)
{
    if (rows.empty()) { std::cout << "\n(No hay filas de rutina)\n"; return; }
    int total = (int(rows.size()));
    int ok = 0;
    double sumBest = 0.0;
    for (auto& r : rows) { if (r.success) ++ok; sumBest += r.bestScore; }
    double avgBest = sumBest / std::max(1, total);

    std::cout << "\n========== RESUMEN RUTINA ==========\n";
    std::cout << "Total (gestos x fases): " << total
              << " | Exitos: " << ok << " (" << (ok * 100 / std::max(1,total)) << "%)\n";
    std::cout << "Score medio (mejor por fila): " << avgBest << "\n";
    std::cout << "------------------------------------\n";
    for (auto& r : rows) {
        std::cout << "[Fase " << r.phase << "] "
                  << r.gesture << "  fuerza=" << int(r.forcePct * 100) << "%  "
                  << (r.success ? "OK" : "FAIL")
                  << "  intentos=" << r.attempts
                  << "  best=" << r.bestScore
                  << "  t=" << r.timeToSuccessMs << "ms\n";
    }
    std::cout << "====================================\n";
}

// =============================================================================
// Utilidades CLI y comparacion en vivo
// =============================================================================

bool captureVector(bool rightHand, std::vector<double>& out){
    HandPose pose;
    if(!CaptureMedianPose(rightHand, pose, 5, 50)) return false;
    return ExtractFeaturesFromPoseString(pose.ToString(), out);
}

// Espera hasta que el score >= threshold durante holdMs; devuelve metricas
bool WaitForHold(bool rightHand,
                 const TemplateStats& T,
                 double threshold,
                 double kTol,
                 int holdMs,
                 int timeLimitMs,
                 double* bestScoreOut,        // OUT: mejor score visto
                 int* timeToSuccessOutMs)     // OUT: ms hasta primer exito (o -1)
{
    if (bestScoreOut) *bestScoreOut = 0.0;
    if (timeToSuccessOutMs) *timeToSuccessOutMs = -1;

    auto t0 = std::chrono::steady_clock::now();
    auto streakStart = t0;
    bool inStreak=false;

    double bestSeen = 0.0;

    while (true) {
        std::vector<double> x;
        if (!captureVector(rightHand, x)) continue;

        double s = ScoreToTemplate(x, T, kTol, 3.0);
        if (s > bestSeen) bestSeen = s;

        if (s >= threshold) {
            if (!inStreak) { inStreak=true; streakStart=std::chrono::steady_clock::now(); }
            auto now = std::chrono::steady_clock::now();
            int elapsed = (int)std::chrono::duration_cast<std::chrono::milliseconds>(now - streakStart).count();
            if (elapsed >= holdMs) {
                if (bestScoreOut) *bestScoreOut = bestSeen;
                if (timeToSuccessOutMs) {
                    int tOK = (int)std::chrono::duration_cast<std::chrono::milliseconds>(now - t0).count();
                    *timeToSuccessOutMs = tOK;
                }
                return true;
            }
        } else {
            inStreak=false;
        }

        auto now = std::chrono::steady_clock::now();
        int total = (int)std::chrono::duration_cast<std::chrono::milliseconds>(now - t0).count();
        if (total >= timeLimitMs) {
            if (bestScoreOut) *bestScoreOut = bestSeen;
            return false;
        }
    }
}

// =============================== MAIN / CLI =================================

int main(){
    std::cout<<"SenseGlove Routine Comparer (plantillas mean/stdev + rutina por fases)\n";
    if(!InitializeSystem()) return 1;

    bool useRightHand = true;
    if (HandLayer::GlovesConnected()==1) useRightHand = HandLayer::GetFirstGloveHandedness();
    else useRightHand = HandLayer::DeviceConnected(true) ? true : false;
    std::cout<<"Usando mano "<<(useRightHand?"derecha":"izquierda")<<". Dataset: "<<kOutputCSV<<"\n";

    double similarityThreshold = 0.88;
    double kTol = 1.0;

    std::cout<<"\nComandos:\n"
             <<"  record                -> guardar gesto (pide dificultad)\n"
             <<"  recordx <N>           -> guardar N repeticiones (pide dificultad)\n"
             <<"  list                  -> listar gestos\n"
             <<"  try                   -> probar un gesto\n"
             <<"  routine               -> ejecutar rutina (3 gestos; fuerza 0%->25%->50%)\n"
             <<"  threshold [v]         -> ver/cambiar umbral\n"
             <<"  ktol [v]              -> ver/cambiar k de tolerancia\n"
             <<"  help / exit\n";

    auto entries = LoadEntries(kOutputCSV);

    std::string cmd;
    while(true){
        std::cout<<"\n> ";
        if(!std::getline(std::cin,cmd)) break;
        cmd = trim(cmd);
        if(cmd.empty()) continue;
        if(cmd=="exit"||cmd=="quit") break;

        if(cmd=="help"){
            std::cout<<"record, recordx <N>, list, try, routine, threshold [v], ktol [v], help, exit\n";
            continue;
        }

        if(cmd.rfind("threshold",0)==0){
            std::string r = trim(cmd.substr(9));
            if(r.empty()) std::cout<<"Umbral = "<<similarityThreshold<<"\n";
            else{
                try { double v=std::stod(r); if(v>0 && v<=1){ similarityThreshold=v; std::cout<<"Nuevo umbral="<<v<<"\n";} else std::cout<<"rango (0,1]\n"; }
                catch(...){ std::cout<<"Uso: threshold 0.9\n"; }
            } continue;
        }

        if(cmd.rfind("ktol",0)==0){
            std::string r = trim(cmd.substr(4));
            if(r.empty()) std::cout<<"kTol = "<<kTol<<"\n";
            else{
                try { double v=std::stod(r); if(v>0){ kTol=v; std::cout<<"Nuevo k="<<v<<"\n";} else std::cout<<"k>0\n"; }
                catch(...){ std::cout<<"Uso: ktol 1.5\n"; }
            } continue;
        }

        if(cmd=="list"){
            auto names = ListGestureNames(kOutputCSV);
            if(names.empty()){ std::cout<<"(no hay gestos)\n"; continue; }
            // mostrar dificultad media
            std::map<std::string, std::pair<int,int>> agg; // name -> (sum,count)
            for(auto& e:entries) agg[e.name].first += e.difficulty, agg[e.name].second++;
            std::cout<<"Gestos:\n";
            for(auto& n:names){
                auto it=agg.find(n);
                int avg = (it==agg.end()||it->second.second==0)? 0 : (it->second.first / it->second.second);
                std::cout<<" - "<<n<<"   (dif aprox "<<avg<<")\n";
            }
            continue;
        }

        if(cmd.rfind("recordx",0)==0){
            int reps=0; try{ reps=std::stoi(trim(cmd.substr(7))); }catch(...){ reps=0; }
            if(reps<=0){ std::cout<<"Uso: recordx 5\n"; continue; }
            std::cout<<"Nombre del gesto: ";
            std::string g; if(!std::getline(std::cin,g) || trim(g).empty()){ std::cout<<"Cancelado.\n"; continue; }
            g=trim(g);
            std::cout<<"Dificultad (entero >=1): ";
            std::string ds; if(!std::getline(std::cin,ds)) ds="1";
            int d=1; try{ d=std::max(1, std::stoi(trim(ds))); }catch(...){ d=1; }

            for(int i=1;i<=reps;++i){
                std::cout<<"Captura "<<i<<"/"<<reps<<"... (mantener postura)\n";
                HandPose pose;
                if(!CaptureMedianPose(useRightHand, pose, 5, 50)){ std::cout<<"Fallo de captura.\n"; break; }
                SaveGesture(kOutputCSV, g, d, pose.ToString());
                entries.push_back({g,d,pose.ToString()});
                std::cout<<"Guardado.\n";
            }
            continue;
        }

        if(cmd=="record"){
            std::cout<<"Nombre del gesto: ";
            std::string g; if(!std::getline(std::cin,g) || trim(g).empty()){ std::cout<<"Cancelado.\n"; continue; }
            g=trim(g);
            std::cout<<"Dificultad (entero >=1): ";
            std::string ds; if(!std::getline(std::cin,ds)) ds="1";
            int d=1; try{ d=std::max(1, std::stoi(trim(ds))); }catch(...){ d=1; }

            std::cout<<"Mantener la postura... (ventana 5 muestras)\n";
            HandPose pose;
            if(!CaptureMedianPose(useRightHand, pose, 5, 50)){ std::cout<<"No se pudo capturar.\n"; continue; }
            SaveGesture(kOutputCSV, g, d, pose.ToString());
            entries.push_back({g,d,pose.ToString()});
            std::cout<<"Gesto '"<<g<<"' (dif "<<d<<") guardado.\n";
            continue;
        }

        if(cmd=="try"){
            auto names = ListGestureNames(kOutputCSV);
            if (names.empty()){ std::cout<<"(no hay gestos)\n"; continue; }
            std::cout<<"Elige un gesto:\n";
            for(size_t i=0;i<names.size();++i) std::cout<<" ["<<i<<"] "<<names[i]<<"\n";
            std::cout<<"Indice: ";
            std::string sidx; if(!std::getline(std::cin,sidx)) continue;
            int idx=-1; try{ idx=std::stoi(sidx);}catch(...){ idx=-1; }
            if(idx<0 || idx>=(int)names.size()){ std::cout<<"Indice invalido.\n"; continue; }

            std::string chosen = names[(size_t)idx];
            TemplateStats T;
            if(!BuildGestureTemplate(entries, chosen, T)){
                std::cout<<"No hay muestras validas para '"<<chosen<<"'.\n"; continue;
            }
            std::cout<<"Plantilla '"<<chosen<<"': "<<T.n<<" muestras.\n";

            std::cout<<"Coloca la mano en '"<<chosen<<"'... (capturando)\n";
            std::vector<double> x;
            if(!captureVector(useRightHand, x)){ std::cout<<"No se pudo capturar o parsear.\n"; continue; }

            double score = ScoreToTemplate(x, T, kTol, 3.0);
            std::cout<<"Score="<<score<<" (umbral="<<similarityThreshold<<")\n";
            std::cout<<(score>=similarityThreshold? "Correcto\n":"Incorrecto\n");
            continue;
        }

        if(cmd=="routine"){
            // 1) seleccionar 3 gestos por dificultad media ascendente
            auto names = ListGestureNames(kOutputCSV);
            if (names.size() < 3){ std::cout<<"Se requieren 3 o mas gestos en el CSV.\n"; continue; }

            struct GD { std::string name; double avg=0; int count=0; };
            std::vector<GD> arr;
            for (auto& n : names){
                long sum=0; int cnt=0;
                for (auto& e: entries) if (e.name==n){ sum+=e.difficulty; cnt++; }
                if (cnt>0) arr.push_back({n, double(sum)/cnt, cnt});
            }
            if (arr.size() < 3){ std::cout<<"Datos insuficientes.\n"; continue; }
            std::sort(arr.begin(), arr.end(), [](const GD& a, const GD& b){
                if (a.avg==b.avg) return a.name<b.name;
                return a.avg<b.avg;
            });

            std::vector<std::string> routine;
            for (size_t i=0; i<3 && i<arr.size(); ++i) routine.push_back(arr[i].name);

            std::cout<<"Rutina (3 gestos, facil->dificil):\n";
            for (size_t i=0;i<routine.size();++i)
                std::cout<<"  "<<(i+1)<<") "<<routine[i]<<" (dif aprox "<<arr[i].avg<<")\n";

            // 2) fases de fuerza aplicadas a TODA la rutina
            const std::vector<float> forceSchedule = { 0.00f, 0.25f, 0.50f }; // 0%, 25%, 50%
            const int holdMs = 300;
            const int timeoutMs = 15000;

            std::vector<RoutineResult> rows;
            const std::string routineId = NowISO8601(); // id de sesion

            for (size_t phase=0; phase<forceSchedule.size(); ++phase){
                float f = forceSchedule[phase];
                std::cout<<"\n===============================\n";
                std::cout<<"FASE "<<(phase+1)<<"/"<<forceSchedule.size()
                         <<"  Fuerza="<<(int)(f*100)<<"%\n";
                std::cout<<"===============================\n";

                ApplyForce(useRightHand, f);

                for (size_t gi=0; gi<routine.size(); ++gi){
                    const std::string& g = routine[gi];
                    TemplateStats T;
                    if(!BuildGestureTemplate(entries, g, T)){
                        std::cout<<"(saltando) No hay plantilla valida para "<<g<<"\n";
                        rows.push_back({NowISO8601(), routineId, g, int(phase+1), f,
                                        false, 0, 0.0, -1, similarityThreshold, kTol});
                       
                        continue;
                    }

                    std::cout<<"\nGesto "<<(gi+1)<<"/"<<routine.size()<<": "<<g<<"\n";
                    int attempts = 0; bool success=false; double bestScore=0.0; int ttsMs=-1;

                    while (true){
                        attempts++;
                        double bestTry=0.0; int ttsTry=-1;
                        bool ok = WaitForHold(useRightHand, T, similarityThreshold, kTol,
                                              holdMs, timeoutMs, &bestTry, &ttsTry);
                        if (bestTry > bestScore) bestScore = bestTry;

                        if (ok){
                            success = true;
                            if (ttsMs < 0) ttsMs = ttsTry; // tiempo al primer exito
                            std::cout<<"Superado.\n";
                            break;
                        } else {
                            std::cout<<"No superado (timeout). Reintentar este gesto? [y/N]: ";
                            std::string ans; std::getline(std::cin, ans);
                            if (ans=="y"||ans=="Y") {// --- RESET DE TENSORES EN CADA REINTENTO ---
                                StopForce(useRightHand);  
                                std::this_thread::sleep_for(std::chrono::milliseconds(700)); 
                                ApplyForce(useRightHand, f); 
                                std::cout << "(Tensores reseteados antes del reintento)\n";
                                continue;
                            } else { std::cout<<"Marcado como no superado.\n"; break; }
                        }
                    }

                    rows.push_back({NowISO8601(), routineId, g, int(phase+1), f,
                                    success, attempts, bestScore, ttsMs,
                                    similarityThreshold, kTol});
                    // --- Reset entre gestos ---
                    StopForce(useRightHand);                      
                    std::this_thread::sleep_for(std::chrono::milliseconds(700)); 
                    ApplyForce(useRightHand, f);                  
                    std::cout << "(Tensores reseteados antes del siguiente gesto)\n";

                }

                StopForce(useRightHand);
                std::cout<<"(Fuerza a 0%)\n";
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }

            // Resumen + CSV
            PrintRoutineSummary(rows);
            AppendRoutineLogCSV("routine_log.csv", rows);
            std::cout<<"Log guardado en routine_log.csv\n";
            continue;
        }

        // atajo: escribir un nombre -> guardar 1 muestra pidiendo dificultad
        {
            std::string g = cmd;
            std::cout<<"Dificultad (entero >=1) para '"<<g<<"': ";
            std::string ds; if(!std::getline(std::cin,ds)) ds="1";
            int d=1; try{ d=std::max(1, std::stoi(trim(ds))); }catch(...){ d=1; }

            std::cout<<"Mantener la postura para '"<<g<<"'... (ventana 5)\n";
            HandPose pose;
            if(!CaptureMedianPose(useRightHand, pose, 5, 50)){ std::cout<<"No se pudo capturar.\n"; continue; }
            SaveGesture(kOutputCSV, g, d, pose.ToString());
            entries.push_back({g,d,pose.ToString()});
            std::cout<<"Gesto '"<<g<<"' (dif "<<d<<") guardado.\n";
        }
    }

    StopForce(useRightHand);
    std::cout<<"Saliendo.\n";
    return 0;
}
