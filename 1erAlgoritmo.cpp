#include <bits/stdc++.h>
#include <chrono>
using namespace std;

struct Avion {
    int Ek, Pk, Lk;        // Tiempo temprano, preferente, tardío
    double Ck, Ci;          // Penalización bajo y sobre preferente
    vector<double> tau;     // tau[j] = separación mínima desde este avión hacia j
};

int D, numPistas;
vector<Avion> aviones;
vector<double> Tk;          // Tiempo asignado a cada avión (-1 si no asignado)
vector<int> pista;          // Pista asignada a cada avión (-1 si no asignado)
double mejorCosto = 1e18;
vector<double> mejorT;
vector<int> mejorPista;
long long nodosExplorados = 0;
long long podasCosto = 0;
long long podasFactibilidad = 0;

chrono::time_point<chrono::high_resolution_clock> tiempoInicio;
//double tiempoLimite = 60.0; // segundos, ajusta según necesites

// ==================== LECTURA DEL ARCHIVO ====================
bool leerArchivo(string nombreArchivo) {
    ifstream archivo(nombreArchivo);
    if (!archivo.is_open()) {
        cerr << "Error al abrir el archivo: " << nombreArchivo << endl;
        return false;
    }

    archivo >> D;
    aviones.resize(D);

    for (int i = 0; i < D; ++i) {
        Avion& avion = aviones[i];
        archivo >> avion.Ek >> avion.Pk >> avion.Lk >> avion.Ck >> avion.Ci;
        avion.tau.resize(D);
        for (int j = 0; j < D; ++j) {
            archivo >> avion.tau[j];
        }
    }

    archivo.close();
    return true;
}

// ==================== CÁLCULO DE COSTO ====================
double calculoCosto(int k, double t) {
    if (t < aviones[k].Pk) {
        return aviones[k].Ck * (aviones[k].Pk - t);
    } else if (t > aviones[k].Pk) {
        return aviones[k].Ci * (t - aviones[k].Pk);
    }
    return 0.0;
}

// ==================== VERIFICACIÓN DE FACTIBILIDAD ====================
bool esFactible(int k, double t, int r) {
    if (t < aviones[k].Ek || t > aviones[k].Lk) return false;

    for (int i = 0; i < D; ++i) {
        if (i == k || Tk[i] < 0 || pista[i] != r) continue;

        // i aterrizó antes que k
        if (Tk[i] <= t && t < Tk[i] + aviones[i].tau[k]) return false;
        // k aterriza antes que i
        if (t < Tk[i] && Tk[i] < t + aviones[k].tau[i]) return false;
    }
    return true;
}

// ==================== HEURÍSTICA MRV ====================
// Selecciona el avión no asignado con la ventana más estrecha
int seleccionarAvion() {
    int mejor = -1;
    int menorVentana = INT_MAX;

    for (int i = 0; i < D; ++i) {
        if (Tk[i] >= 0) continue; // ya asignado
        int ventana = aviones[i].Lk - aviones[i].Ek;
        if (ventana < menorVentana) {
            menorVentana = ventana;
            mejor = i;
        }
    }
    return mejor;
}

// ==================== GENERACIÓN DE TIEMPOS RELEVANTES ====================
// Solo probamos los puntos críticos del dominio en vez de todos los enteros
vector<int> tiemposRelevantes(int k, int r) {
    set<int> tiempos;

    // Puntos base
    tiempos.insert(aviones[k].Ek);
    tiempos.insert(aviones[k].Pk);
    tiempos.insert(aviones[k].Lk);

    // Puntos forzados por aviones ya asignados en la misma pista
    for (int i = 0; i < D; ++i) {
        if (i == k || Tk[i] < 0 || pista[i] != r) continue;
        int ti = (int)Tk[i];

        // Tiempo mínimo para k si aterriza DESPUÉS de i
        int despues = ti + (int)ceil(aviones[i].tau[k]);
        if (despues >= aviones[k].Ek && despues <= aviones[k].Lk)
            tiempos.insert(despues);

        // Tiempo máximo para k si aterriza ANTES de i
        int antes = ti - (int)ceil(aviones[k].tau[i]);
        if (antes >= aviones[k].Ek && antes <= aviones[k].Lk)
            tiempos.insert(antes);
    }

    // Ordenar por cercanía al preferente (menor costo primero)
    vector<int> result(tiempos.begin(), tiempos.end());
    sort(result.begin(), result.end(), [&](int a, int b) {
        return abs(a - aviones[k].Pk) < abs(b - aviones[k].Pk);
    });

    return result;
}

// ==================== BACKTRACKING PRINCIPAL ====================
void backtracking(int asignados, double costoActual) {
    nodosExplorados++;

    //double transcurrido = chrono::duration<double>(
        //chrono::high_resolution_clock::now() - tiempoInicio).count();
    //if (transcurrido >= tiempoLimite) return;
    // Poda por costo
    if (costoActual >= mejorCosto) {
        podasCosto++;
        return;
    }

    // Solución completa encontrada
    if (asignados == D) {
        mejorCosto = costoActual;
        mejorT     = Tk;
        mejorPista = pista;
        cout << "  Nueva solucion: " << fixed << setprecision(2) << mejorCosto << endl;
        return;
    }

    // Seleccionar avión con menor ventana (MRV)
    int k = seleccionarAvion();
    if (k == -1) return;

    for (int r = 0; r < numPistas; ++r) {
        vector<int> tiempos = tiemposRelevantes(k, r);

        for (int t : tiempos) {
            // Poda: si este avión solo ya supera el mejor, saltar
            double costoK = calculoCosto(k, t);
            if (costoActual + costoK >= mejorCosto) {
                podasCosto++;
                continue;
            }

            if (esFactible(k, t, r)) {
                Tk[k]    = t;
                pista[k] = r;
                backtracking(asignados + 1, costoActual + costoK);
                Tk[k]    = -1;
                pista[k] = -1;
            } else {
                podasFactibilidad++;
            }
        }
    }
}

// ==================== FUNCIÓN PRINCIPAL ====================
int main(int argc, char* argv[]) {
    string archivo = (argc > 1) ? argv[1] : "case1.txt";
    numPistas      = (argc > 2) ? atoi(argv[2]) : 1;
    
    if (!leerArchivo(archivo)) return 1;

    // Inicializar estado
    Tk.assign(D, -1.0);
    pista.assign(D, -1);
    mejorCosto        = 1e18;
    nodosExplorados   = 0;
    podasCosto        = 0;
    podasFactibilidad = 0;

    cout << "=== BACKTRACKING CRONOLOGICO ===" << endl;
    cout << "Archivo: " << archivo << endl;
    cout << "Pistas:  " << numPistas << endl;
    cout << "Aviones: " << D << endl;  // ahora D ya fue leído
    cout << "--------------------------------" << endl;

    tiempoInicio = chrono::high_resolution_clock::now();
    backtracking(0, 0.0);
    auto fin      = chrono::high_resolution_clock::now();
    auto duracion = chrono::duration_cast<chrono::milliseconds>(fin - tiempoInicio);

    // Mostrar resultados
    cout << "\n=== RESULTADOS ===" << endl;
    if (mejorCosto >= 1e17) {
        cout << "No se encontro solucion factible." << endl;
    } else {
        cout << "Mejor costo: " << fixed << setprecision(2) << mejorCosto << endl;
        cout << "\nAvion\tPista\tEk\tPk\tLk\tT asignado\tCosto" << endl;
        double costoTotal = 0;
        for (int i = 0; i < D; ++i) {
            double costo = calculoCosto(i, mejorT[i]);
            costoTotal += costo;
            cout << i + 1 << "\t"
                 << mejorPista[i] + 1 << "\t"
                 << aviones[i].Ek << "\t"
                 << aviones[i].Pk << "\t"
                 << aviones[i].Lk << "\t"
                 << fixed << setprecision(2) << mejorT[i] << "\t\t"
                 << costo << endl;
        }
        cout << "\nCosto total verificado: " << fixed << setprecision(2) << costoTotal << endl;
    }

    cout << "\n=== METRICAS ===" << endl;
    cout << "Nodos explorados:       " << nodosExplorados << endl;
    cout << "Podas por costo:        " << podasCosto << endl;
    cout << "Podas por factibilidad: " << podasFactibilidad << endl;
    //cout << "Tiempo de ejecucion:    " << duracion.count() << " ms" << endl;

    return 0;
}