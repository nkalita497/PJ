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

Result dijkstra_shortest(const Graph& G){
    Result R;
    auto t0 = chrono::steady_clock::now();
    const long long INF = (1LL<<60);

    vector<long long> dist(G.n, INF);
    vector<int> parent(G.n, -1);
    using State = pair<long long,int>;
    priority_queue<State, vector<State>, greater<State>> pq;dist[G.s]=0; pq.push({0,G.s});
    while(!pq.empty()){
        auto [du,u]=pq.top(); pq.pop();
        if(du!=dist[u]) continue;
        R.visited++;
        if(u==G.t) break;
        for(auto [v,w]: G.adj[u]){
            long long nd = du + w;
            if(nd < dist[v]){
                dist[v]=nd;
                parent[v]=u;
                pq.push({nd,v});
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

    vector<long long> g(G.n, INF);
    vector<int> parent(G.n, -1);
    using State = pair<long long,int>;
    priority_queue<State, vector<State>, greater<State>> open;

    g[G.s]=0; open.push({0,G.s});
    while(!open.empty()){
        auto [f,u]=open.top(); open.pop();
        if(f!=g[u]) continue;
        R.visited++;
        if(u==G.t) break;
        for(auto [v,w]: G.adj[u]){
            long long ng = g[u] + w;
            if(ng < g[v]){
                g[v]=ng;
                parent[v]=u;
                open.push({ng, v});
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

int main(int argc, char** argv){
    ios::sync_with_stdio(false);
    cin.tie(nullptr);

    string algo, input;

    if (argc == 1) {
        cout << "Wybierz algorytm (bfs/dijkstra/astar): ";
        cin >> algo;
        cout << "Podaj nazwę pliku (np. graph.txt): ";
        cin >> input;
    } else {
        for(int i=1;i<argc;i++){
            string a = argv[i];
            if(a=="--algo" && i+1<argc) algo = argv[++i];
            else if(a=="--input" && i+1<argc) input = argv[++i];
        }
    }

    if(algo.empty() || input.empty()){
        cerr << "Użycie: sp --algo bfs|dijkstra|astar --input plik.txt\n";
        return 1;
    }

    Graph G;
    if(!read_graph(input, G)) return 2;

    Result R;
    if(algo=="bfs"){
        if(G.weighted){
            cerr << "BFS tylko dla UNWEIGHTED (nieważone). Użyj dijkstra/astar.\n";
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

    if(!R.found){
        cout << "NO PATH\n";
    } else {
        cout << "COST: " << R.cost << "\n";
        cout << "PATH: ";
        for(size_t i=0;i<R.path.size();++i){
            if(i) cout << " ";
            cout << R.path[i];
        }
        cout << "\n";
    }
    cout << "VISITED: " << R.visited << "\n";
    cout << "TIME_MS: " << R.ms << "\n";

    return 0;
}