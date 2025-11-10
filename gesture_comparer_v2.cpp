// routine_comparer.cpp — SenseGlove: Rutinas por gestos con fases de fuerza
//
// Novedades respecto a gesture_comparer_v2.cpp:
//  - CSV con dificultad: "gesture_name;difficulty;pose_string"
//    (compatible con el formato antiguo "gesture_name;pose_string")
//  - Comando `routine`: selecciona 3 gestos (más fáciles primero), y para cada uno:
//        Fase 1 -> reconocer gesto (score>=umbral, mantener 300ms)  [fuerza L1]
//        Fase 2 -> repetir con mayor fuerza                         [fuerza L2]
//        Fase 3 -> repetir con mayor fuerza                         [fuerza L3]
//    Luego pasa al siguiente gesto. Al final, fuerza=0 (stop).
//  - `record` y `recordx` preguntan dificultad (entero).
//
// Compilación típica (depende de tu entorno SGCore):
//   g++ routine_comparer.cpp -std=c++17 -lSGCoreCpp -pthread -o routine_comparer
//
// -----------------------------------------------------------------------------
// CAPAS (en un solo archivo por simplicidad didáctica):
//   Persistencia CSV     : ListGestureNames, LoadEntries, SaveGesture
//   Adquisición (SDK)    : InitializeSystem, CaptureMedianPose
//   Normalización        : ExtractFeaturesFromPoseString
//   Plantillas/Comparador: BuildGestureTemplateFromCSV, ScoreToTemplate
//   CLI / Rutinas        : main() con comandos record/try/routine/etc.
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
    int         difficulty = 1;     // entero >=1
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
// Adquisición (SDK)
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
// Normalización (parseo de pose -> 21 features)
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
    return ok >= (out.size()*7)/10; // ≥70% presentes
}

// =============================================================================
// Plantillas y comparador
// =============================================================================

struct TemplateStats {
    std::vector<double> mu;
    std::vector<double> sigma;
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

    T.mu.assign(F, std::numeric_limits<double>::quiet_NaN());
    T.sigma.assign(F, std::numeric_limits<double>::quiet_NaN());
    T.n = used;

    for(size_t i=0;i<F;++i){
        if(count[i]==0) continue;
        double m = sum[i]/double(count[i]);
        double v = std::max(0.0, sum2[i]/double(count[i]) - m*m);
        T.mu[i]=m; T.sigma[i]=std::sqrt(v);
    }
    return true;
}

// score ∈ [0,1]. k escala de tolerancia; eps en grados.
double ScoreToTemplate(const std::vector<double>& x,
                       const TemplateStats& T,
                       double k=1.0, double eps=3.0)
{
    const size_t F=kOrder.size();
    double num=0.0, den=0.0; size_t used=0;
    for(size_t i=0;i<F;++i){
        double mu=T.mu[i], s=T.sigma[i], xi=x[i];
        if(std::isnan(mu)||std::isnan(xi)) continue;
        double tol=std::max(k*s, eps);
        double err=std::abs(xi-mu);
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
// Utilidades CLI
// =============================================================================

bool captureVector(bool rightHand, std::vector<double>& out){
    HandPose pose;
    if(!CaptureMedianPose(rightHand, pose, 5, 50)) return false;
    return ExtractFeaturesFromPoseString(pose.ToString(), out);
}

// Espera hasta que el score ≥ threshold durante holdMs consecutivos (dentro de un límite total)
bool WaitForHold(bool rightHand,
                 const TemplateStats& T,
                 double threshold,
                 double kTol,
                 int holdMs = 300,
                 int timeLimitMs = 15000)
{
    auto t0 = std::chrono::steady_clock::now();
    auto streakStart = t0;
    bool inStreak=false;

    while(true){
        std::vector<double> x;
        if(!captureVector(rightHand, x)) continue;

        double s = ScoreToTemplate(x, T, kTol, 3.0);
        // std::cout << "score="<<s<<"\r" << std::flush;

        if (s >= threshold){
            if(!inStreak){ inStreak=true; streakStart=std::chrono::steady_clock::now(); }
            auto now = std::chrono::steady_clock::now();
            int elapsed = (int)std::chrono::duration_cast<std::chrono::milliseconds>(now-streakStart).count();
            if (elapsed >= holdMs) return true; // conseguido
        } else {
            inStreak=false;
        }

        auto now = std::chrono::steady_clock::now();
        int total = (int)std::chrono::duration_cast<std::chrono::milliseconds>(now-t0).count();
        if (total >= timeLimitMs) return false; // timeout
    }
}

// =============================== MAIN / CLI =================================

int main(){
    std::cout<<"SenseGlove Routine Comparer (μ/σ + rutinas por fases de fuerza)\n";
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
             <<"  try                   -> probar un gesto (μ/σ + score)\n"
             <<"  routine               -> ejecutar rutina (3 gestos; 3 fases de fuerza)\n"
             <<"  threshold [v]         -> ver/cambiar umbral\n"
             <<"  ktol [v]              -> ver/cambiar k de tolerancia (σ)\n"
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
                try { double v=std::stod(r); if(v>0 && v<=1){ similarityThreshold=v; std::cout<<"Nuevo umbral="<<v<<"\n";} else std::cout<<"(0,1]\n"; }
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
            // Mostrar dificultad media
            std::map<std::string, std::pair<int,int>> agg; // name -> (sum,count)
            for(auto& e:entries) agg[e.name].first += e.difficulty, agg[e.name].second++;
            std::cout<<"Gestos:\n";
            for(auto& n:names){
                auto it=agg.find(n);
                int avg = (it==agg.end()||it->second.second==0)? 0 : (it->second.first / it->second.second);
                std::cout<<" - "<<n<<"   (dif≈"<<avg<<")\n";
            }
            continue;
        }

        if(cmd.rfind("recordx",0)==0){
            int reps=0; try{ reps=std::stoi(trim(cmd.substr(7))); }catch(...){ reps=0; }
            if(reps<=0){ std::cout<<"Uso: recordx 5\n"; continue; }
            std::cout<<"Nombre del gesto: ";
            std::string g; if(!std::getline(std::cin,g) || trim(g).empty()){ std::cout<<"Cancelado.\n"; continue; }
            g=trim(g);
            std::cout<<"Dificultad (entero ≥1): ";
            std::string ds; if(!std::getline(std::cin,ds)) ds="1";
            int d=1; try{ d=std::max(1, std::stoi(trim(ds))); }catch(...){ d=1; }

            for(int i=1;i<=reps;++i){
                std::cout<<"Captura "<<i<<"/"<<reps<<"... (mantén postura)\n";
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
            std::cout<<"Dificultad (entero ≥1): ";
            std::string ds; if(!std::getline(std::cin,ds)) ds="1";
            int d=1; try{ d=std::max(1, std::stoi(trim(ds))); }catch(...){ d=1; }

            std::cout<<"Mantén la postura... (ventana 5 muestras)\n";
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
            std::cout<<"Índice: ";
            std::string sidx; if(!std::getline(std::cin,sidx)) continue;
            int idx=-1; try{ idx=std::stoi(sidx);}catch(...){ idx=-1; }
            if(idx<0 || idx>=(int)names.size()){ std::cout<<"Índice inválido.\n"; continue; }

            std::string chosen = names[(size_t)idx];
            TemplateStats T;
            if(!BuildGestureTemplate(entries, chosen, T)){
                std::cout<<"No hay muestras válidas para '"<<chosen<<"'.\n"; continue;
            }
            std::cout<<"Plantilla '"<<chosen<<"': "<<T.n<<" muestras.\n";

            std::cout<<"Coloca la mano en '"<<chosen<<"'... (capturando)\n";
            std::vector<double> x;
            if(!captureVector(useRightHand, x)){ std::cout<<"No se pudo capturar/parsear.\n"; continue; }

            double score = ScoreToTemplate(x, T, kTol, 3.0);
            std::cout<<"Score="<<score<<" (umbral="<<similarityThreshold<<")\n";
            std::cout<<(score>=similarityThreshold? "Correcto\n":"Incorrecto\n");
            continue;
        }

        if(cmd=="routine"){
            // 1) seleccionar 3 gestos por dificultad media ascendente
            auto names = ListGestureNames(kOutputCSV);
            if (names.size()<3){ std::cout<<"Se requieren ≥3 gestos en el CSV.\n"; continue; }

            // dificultad media por gesto
            struct GD { std::string name; double avg=0; int count=0; };
            std::vector<GD> arr;
            for(auto& n:names){
                long sum=0; int cnt=0;
                for(auto& e:entries) if(e.name==n){ sum+=e.difficulty; cnt++; }
                if(cnt>0) arr.push_back({n, double(sum)/cnt, cnt});
            }
            if(arr.size()<3){ std::cout<<"Datos insuficientes.\n"; continue; }
            std::sort(arr.begin(), arr.end(), [](const GD& a, const GD& b){
                if (a.avg==b.avg) return a.name<b.name;
                return a.avg<b.avg;
            });

            std::vector<std::string> routine;
            for(size_t i=0; i<3 && i<arr.size(); ++i) routine.push_back(arr[i].name);

            std::cout<<"Rutina (3 gestos, fácil→difícil):\n";
            for(size_t i=0;i<routine.size();++i) std::cout<<"  "<<(i+1)<<") "<<routine[i]<<" (dif≈"<<arr[i].avg<<")\n";

            // 2) fases de fuerza
            const std::vector<float> forceSchedule = { 0.20f, 0.35f, 0.50f }; // 20%, 35%, 50%
            const int holdMs = 300;

            for (auto& g : routine){
                std::cout<<"\n=== Gesto: "<<g<<" ===\n";
                TemplateStats T;
                if(!BuildGestureTemplate(entries, g, T)){
                    std::cout<<"(saltando) No hay plantilla válida para "<<g<<"\n"; continue;
                }
                for(size_t phase=0; phase<forceSchedule.size(); ++phase){
                    float f = forceSchedule[phase];
                    std::cout<<"Fase "<<(phase+1)<<"/"<<forceSchedule.size()<<": fuerza="<<(int)(f*100)<<"%.\n";
                    ApplyForce(useRightHand, f);
                    std::cout<<"Coloca el gesto y mantenlo "<<holdMs<<"ms por encima del umbral ("<<similarityThreshold<<")...\n";

                    bool ok = WaitForHold(useRightHand, T, similarityThreshold, kTol, holdMs, /*timeout*/15000);
                    if(!ok){
                        std::cout<<"No superado (timeout). ¿Reintentar esta fase? [y/N]: ";
                        std::string ans; std::getline(std::cin, ans);
                        if(ans=="y"||ans=="Y"){ --phase; continue; }
                        else { std::cout<<"Marcado como no superado. Continuamos.\n"; break; }
                    } else {
                        std::cout<<"Superado.\n";
                    }
                }
                StopForce(useRightHand);
                std::cout<<"(Fuerza a 0%)\n";
            }

            std::cout<<"\nRutina finalizada.\n";
            continue;
        }

        // atajo: escribir un nombre -> guardar 1 muestra pidiendo dificultad
        {
            std::string g = cmd;
            std::cout<<"Dificultad (entero ≥1) para '"<<g<<"': ";
            std::string ds; if(!std::getline(std::cin,ds)) ds="1";
            int d=1; try{ d=std::max(1, std::stoi(trim(ds))); }catch(...){ d=1; }

            std::cout<<"Mantén la postura para '"<<g<<"'... (ventana 5)\n";
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
