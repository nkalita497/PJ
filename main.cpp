#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <queue>
#include <climits>
#include <chrono>

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

struct Result {
    long long cost = LLONG_MAX;
    vector<int> path;
    long long visited = 0;
    long long ms = 0;
    bool found = false;
};

vector<int> reconstruct(int s, int t, const vector<int>& parent){
    vector<int> p;
    if(t<0 || t>= (int)parent.size()) return p;
    if(parent[t]==-1 && s!=t) return p;
    for(int v=t; v!=-1; v=parent[v]) p.push_back(v);
    reverse(p.begin(), p.end());
    return p;
}

Result bfs_shortest(const Graph& G){
    Result R;
    auto t0 = chrono::steady_clock::now();

    vector<int> dist(G.n, INT_MAX), parent(G.n,-1);
    queue<int>q;
    dist[G.s]=0; q.push(G.s);
    while(!q.empty()){
        int u=q.front(); q.pop();
        R.visited++;
        if(u==G.t) break;
        for(auto [v,w]: G.adj[u]){
            (void)w;
            if(dist[v]==INT_MAX){
                dist[v]=dist[u]+1;
                parent[v]=u;
                q.push(v);
            }
        }
    }
    if(dist[G.t]!=INT_MAX){
        R.found = true;
        R.cost = dist[G.t];
        R.path = reconstruct(G.s, G.t, parent);
    }
    auto t1 = chrono::steady_clock::now();
    R.ms = chrono::duration_cast<chrono::milliseconds>(t1-t0).count();
    return R;
}

int main() {
    Graph G;
    if (read_graph("graf.txt", G)) {
        std::cout << "Graf wczytany poprawnie.\n";
    } else {
        std::cerr << "Błąd przy wczytywaniu grafu.\n";
    }

    Result R = bfs_shortest(G);
    cout << "BFS" << endl;
    cout << "Odwiedzone: " << R.visited << endl;
    cout << "Czas: " << R.ms << endl;
    cout << "Czy znaleziona: " << R.found << endl;
    if (R.found) {
        cout << "Koszt: " << bfs_shortest(G).cost << endl;
        cout << "Ścieżka: ";
        for(int i=0; i<R.path.size(); i++) {
            if(R.path[i]==1) cout << R.path[i] << " ";
            cout << R.path[i] << " ";
        }
    }

    return 0;
}
