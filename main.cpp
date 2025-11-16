#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <queue>
#include <climits>
#include <chrono>
#include <algorithm>
#include <iomanip>

using namespace std;

struct Graph {
    bool weighted = false;
    int n = 0, m = 0;
    int s = 0, t = 0;
    vector<vector<pair<int, long long>>> adj;
};

struct Result {
    long long cost = -1;
    vector<int> path;
    long long visited = 0;
    long long ms = 0;
    bool found = false;
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

    if (G.n <= 0 || G.m < 0 || G.s < 0 || G.s >= G.n || G.t < 0 || G.t >= G.n) {
        cerr << "Błędne parametry n/m/s/t (n musi być > 0, s i t muszą być w zakresie [0, n-1]).\n";
        return false;
    }

    G.adj.assign(G.n, {});

    for(int i = 0; i < G.m; i++){
        int u, v;
        in >> u >> v;
        if(!in){
            cerr << "Za mało krawędzi w pliku (oczekiwano " << G.m << ").\n";
            return false;
        }
        if(u < 0 || u >= G.n || v < 0 || v >= G.n){
            cerr << "Wierzchołki muszą być w przedziale [0, n-1]. Napotkano krawędź: " << u << " " << v << "\n";
            return false;
        }

        long long w = 1; // Domyślna waga dla grafu nieważonego
        if(G.weighted){
            in >> w;
            if(!in){
                cerr << "Brak wagi krawędzi " << u << " " << v << " (wymagane dla grafu WEIGHTED).\n";
                return false;
            }
            if (w < 0) {
                cerr << "Waga krawędzi nie może być ujemna. Napotkano wagę: " << w << "\n";
                return false;
            }
        }

        G.adj[u].push_back({v, w});
        G.adj[v].push_back({u, w});
    }

    return true;
}

vector<int> reconstruct(int s, int t, const vector<int>& parent){
    vector<int> p;
    if(t<0 || t>= (int)parent.size() || (parent[t] == -1 && s != t)) {
        return p;
    }

    for(int v=t; v!=-1; v=parent[v]) {
        p.push_back(v);
        if (p.size() > parent.size()) {
            p.clear();
            return p;
        }
    }

    reverse(p.begin(), p.end());
    return p;
}

Result bfs_shortest(const Graph& G){
    Result R;
    auto t0 = chrono::steady_clock::now();

    vector<int> dist(G.n, INT_MAX);
    vector<int> parent(G.n,-1);
    queue<int> q;

    dist[G.s] = 0;
    q.push(G.s);

    while(!q.empty()){
        int u = q.front();
        q.pop();
        R.visited++;

        if(u == G.t) break;

        for(auto [v, w] : G.adj[u]){
            (void)w;
            if(dist[v] == INT_MAX){
                dist[v] = dist[u] + 1;
                parent[v] = u;
                q.push(v);
            }
        }
    }

    if(dist[G.t] != INT_MAX){
        R.found = true;
        R.cost = dist[G.t];
        R.path = reconstruct(G.s, G.t, parent);
    }

    auto t1 = chrono::steady_clock::now();
    R.ms = chrono::duration_cast<chrono::milliseconds>(t1-t0).count();
    return R;
}

Result dijkstra_shortest(const Graph& G){
    Result R;
    auto t0 = chrono::steady_clock::now();
    const long long INF = (1LL<<60);

    vector<long long> dist(G.n, INF);
    vector<int> parent(G.n, -1);
    using State = pair<long long, int>;
    priority_queue<State, vector<State>, greater<State>> pq;

    dist[G.s] = 0;
    pq.push({0, G.s});

    while(!pq.empty()){
        auto [du, u] = pq.top();
        pq.pop();

        if(du > dist[u]) continue;
        R.visited++;

        if(u == G.t) break;

        for(auto [v, w] : G.adj[u]){
            long long nd = du + w;
            if(nd < dist[v]){
                dist[v] = nd;
                parent[v] = u;
                pq.push({nd, v});
            }
        }
    }

    if(dist[G.t] != INF){
        R.found = true;
        R.cost = dist[G.t];
        R.path = reconstruct(G.s, G.t, parent);
    }

    auto t1 = chrono::steady_clock::now();
    R.ms = chrono::duration_cast<chrono::milliseconds>(t1-t0).count();
    return R;
}


Result astar_zero(const Graph& G){
    Result R;
    auto t0 = chrono::steady_clock::now();
    const long long INF = (1LL<<60);

    vector<long long> g(G.n, INF); // g(u) - faktyczny koszt od S do U
    vector<int> parent(G.n, -1);
    using State = pair<long long, int>;
    priority_queue<State, vector<State>, greater<State>> open;

    auto heuristic = [](int u, int t) {
        (void)u; (void)t;
        return 0LL;
    };

    g[G.s] = 0;
    open.push({g[G.s] + heuristic(G.s, G.t), G.s});

    while(!open.empty()){
        auto [f, u] = open.top();
        open.pop();

        if(f != g[u] + heuristic(u, G.t)) continue;

        R.visited++;
        if(u == G.t) break;

        for(auto [v, w] : G.adj[u]){
            long long ng = g[u] + w; // Nowy koszt do v
            if(ng < g[v]){
                g[v] = ng;
                parent[v] = u;
                open.push({ng + heuristic(v, G.t), v});
            }
        }
    }

    if(g[G.t] != INF){
        R.found = true;
        R.cost = g[G.t];
        R.path = reconstruct(G.s, G.t, parent);
    }

    auto t1 = chrono::steady_clock::now();
    R.ms = chrono::duration_cast<chrono::milliseconds>(t1-t0).count();
    return R;
}

void print_result_row(const string& name, const Result& R) {
    cout << left << setw(10) << name;

    if (!R.found) {
        cout << setw(8) << "NO"
             << setw(10) << "-"
             << setw(12) << R.visited
             << setw(10) << R.ms
             << "\n";
    } else {
        cout << setw(8)  << "YES"
             << setw(10) << R.cost
             << setw(12) << R.visited
             << setw(10) << R.ms
             << "\n";
    }
}

void compare_algorithms(const Graph& G) {
    cout << "--- PORÓWNANIE ALGORYTMÓW NA TYM SAMYM GRAFIE ---\n";
    cout << "Graf (N/M/S/T): " << G.n << "/" << G.m << "/" << G.s << "/" << G.t << "\n\n";

    Result Rbfs, Rdij, Rast;
    bool can_bfs = !G.weighted;

    if (can_bfs) {
        Rbfs = bfs_shortest(G);
    }

    Rdij = dijkstra_shortest(G);
    Rast = astar_zero(G);

    cout << left << setw(10) << "ALGO"
         << setw(8)  << "FOUND"
         << setw(10) << "COST"
         << setw(12) << "VISITED"
         << setw(10) << "TIME_MS"
         << "\n";

    cout << string(50, '-') << "\n";

    if (can_bfs) {
        print_result_row("bfs", Rbfs);
    } else {
        cout << left << setw(10) << "bfs"
             << setw(8)  << "N/A"
             << setw(10) << "-"
             << setw(12) << "-"
             << setw(10) << "-"
             << "\n";
    }

    print_result_row("dijkstra", Rdij);
    print_result_row("astar",    Rast);
}

int main(int argc, char** argv){
    ios::sync_with_stdio(false);
    cin.tie(nullptr);

    string algo, input;
    bool compare_mode = false;

    if (argc < 2) {
        cout << "Wybierz algorytm (bfs/dijkstra/astar lub compare): ";
        cin >> algo;
        cout << "Podaj nazwę pliku (np. graph.txt) : ";
        cin >> input;
        if (algo == "compare") compare_mode = true;
    } else {
        for(int i = 1; i < argc; i++){
            string a = argv[i];
            if(a == "--algo" && i + 1 < argc) algo = argv[++i];
            else if(a == "--input" && i + 1 < argc) input = argv[++i];
            else if(a == "--compare") compare_mode = true;
        }
    }

    if (input.empty()){
        cerr << "Użycie: sp --algo bfs|dijkstra|astar [--compare] --input plik.txt\n";
        return 1;
    }

    Graph G;
    if(!read_graph(input, G)) return 2;

    if (compare_mode) {
        compare_algorithms(G);
        return 0;
    }

    if(algo.empty()){
        cerr << "Użycie: sp --algo bfs|dijkstra|astar --input plik.txt\n";
        return 1;
    }

    Result R;

    if(algo=="bfs"){
        if(G.weighted){
            cerr << "BFS działa poprawnie tylko dla UNWEIGHTED (lub ważonych jedynkami). Dla grafu ważonego użyj dijkstra/astar.\n";
            return 3;
        }
        R = bfs_shortest(G);
    } else if(algo=="dijkstra"){
        R = dijkstra_shortest(G);
    } else if(algo=="astar"){
        R = astar_zero(G);
    } else {
        cerr << "Nieznany algorytm: " << algo << "\n";
        return 4;
    }

    cout << "--- WYNIK DLA ALGO: " << algo << " ---\n";
    cout << "Graf (N/M/S/T): " << G.n << "/" << G.m << "/" << G.s << "/" << G.t << "\n";

    if(!R.found){
        cout << "STATUS: NO PATH\n";
    } else {
        cout << "STATUS: PATH FOUND\n";
        cout << "COST: " << R.cost << "\n";
        cout << "PATH (" << R.path.size() << " nodes): ";
        for(size_t i = 0; i < R.path.size(); ++i){
            if(i) cout << " ";
            cout << R.path[i];
        }
        cout << "\n";
    }

    cout << "METRICS:\n";
    cout << "  VISITED: " << R.visited << "\n";
    cout << "  TIME_MS: " << R.ms << "\n";

    return 0;
}