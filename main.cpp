#include <iostream>
#include <vector>
#include <string>
#include <fstream>
using namespace std;

struct Graph {
    bool weighted = false;
    int n = 0, m = 0;
    int s = 0, t = 0;
    vector<vector<pair<int,int>>> adj;
};

bool read_graph(const string& filename, Graph& G) {
    ifstream in(filename);
    if(!in) {
        cerr << "Nie mogę otworzyć pliku: " << filename << "\n";
        return false;
    }

    string kind;
    in >> kind;
    if(kind == "UNWEIGHTED") G.weighted = false;
    else if(kind == "WEIGHTED") G.weighted = true;
    else {
        cerr << "Pierwsza linia musi być UNWEIGHTED albo WEIGHTED\n";
        return false;
    }

    in >> G.n >> G.m;
    in >> G.s >> G.t;

    // Poprawiony warunek sprawdzający parametry
    if (G.n <= 0 || G.m < 0 || G.s < 0 || G.s >= G.n || G.t < 0 || G.t >= G.n) {
        cerr << "Błędne parametry n/m/s/t\n";
        return false;
    }

    G.adj.assign(G.n, {});

    for(int i = 0; i < G.m; i++){
        int u, v;
        in >> u >> v;
        if(!in){
            cerr << "Za mało krawędzi w pliku.\n";
            return false;
        }
        if(u < 0 || u >= G.n || v < 0 || v >= G.n){
            cerr << "Wierzchołki muszą być w przedziale [0, n-1]\n";
            return false;
        }

        if(G.weighted){
            int w;
            in >> w;
            if(!in){
                cerr << "Brak wagi krawędzi.\n";
                return false;
            }
            G.adj[u].push_back({v, w});
            G.adj[v].push_back({u, w});
        } else {
            G.adj[u].push_back({v, 1});
            G.adj[v].push_back({u, 1});
        }
    }

    return true;
}

int main() {
    Graph G;
    if (read_graph("graf.txt", G)) {
        std::cout << "Graf wczytany poprawnie.\n";
    } else {
        std::cerr << "Błąd przy wczytywaniu grafu.\n";
    }
    return 0;
}
