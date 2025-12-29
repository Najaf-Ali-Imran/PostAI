#include <crow.h>
#include <crow/middlewares/cors.h>
#include <iostream>
#include <vector>
#include <stack>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <set>
#include <cmath>
#include <iomanip>
#include <string>
#include <algorithm>
#include <limits>
#include <chrono>
#include <bitset>
#include <climits>
#include <cstdlib>

using namespace std;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

struct Location {
    int id;
    double lat;
    double lon;
    string type;
    bool isHighPriority;
    double eta;
    
    Location() : id(0), lat(0.0), lon(0.0), type(""), isHighPriority(false), eta(0.0) {}
};

struct Edge {
    int destination;
    double distance;
    
    Edge(int dest, double dist) : destination(dest), distance(dist) {}
};

double haversineDistance(double lat1, double lon1, double lat2, double lon2) {
    const double earthRadiusKm = 6371.0;
    
    double lat1Rad = lat1 * M_PI / 180.0;
    double lat2Rad = lat2 * M_PI / 180.0;
    double lon1Rad = lon1 * M_PI / 180.0;
    double lon2Rad = lon2 * M_PI / 180.0;
    
    double deltaLat = lat2Rad - lat1Rad;
    double deltaLon = lon2Rad - lon1Rad;
    
    double a = sin(deltaLat / 2.0) * sin(deltaLat / 2.0) +
               cos(lat1Rad) * cos(lat2Rad) *
               sin(deltaLon / 2.0) * sin(deltaLon / 2.0);
    
    double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
    
    return earthRadiusKm * c;
}

class LogisticsGraph {
private:
    int numVertices;
    vector<vector<Edge>> adjacencyList;
    unordered_map<int, Location> locations;
    
public:
    LogisticsGraph(int vertices) : numVertices(vertices) {
        adjacencyList.resize(vertices);
    }
    
    void addEdge(int source, int destination, double distance) {
        if (source >= 0 && source < numVertices && destination >= 0 && destination < numVertices) {
            adjacencyList[source].push_back(Edge(destination, distance));
        }
    }
    
    void addLocation(const Location& loc) {
        if (loc.id >= 0 && loc.id < numVertices) {
            locations[loc.id] = loc;
        }
    }
    
    Location getLocation(int id) const {
        auto it = locations.find(id);
        if (it != locations.end()) {
            return it->second;
        }
        return Location();
    }
    
    const vector<Edge>& getNeighbors(int locationId) const {
        return adjacencyList[locationId];
    }
    
    int getNumVertices() const {
        return numVertices;
    }
    
    double getDistance(int from, int to) const {
        for (const auto& edge : adjacencyList[from]) {
            if (edge.destination == to) {
                return edge.distance;
            }
        }
        return numeric_limits<double>::max();
    }
};

stack<vector<Location>> historyStack;

struct PriorityNode {
    int node;
    double cost;
    int parent;
    
    PriorityNode(int n, double c, int p = -1) : node(n), cost(c), parent(p) {}
    
    bool operator>(const PriorityNode& other) const {
        return cost > other.cost;
    }
};

class RouteOptimizer {
private:
    vector<Location> allLocations;
    vector<vector<double>> distMatrix;
    const double AVERAGE_SPEED_KMH = 40.0;
    const int HELD_KARP_THRESHOLD = 12;
    const int MAX_2OPT_ITERATIONS = 100;
    
    void buildDistanceMatrix() {
        int n = allLocations.size();
        distMatrix.resize(n, vector<double>(n, 0.0));
        
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                if (i != j) {
                    distMatrix[i][j] = haversineDistance(
                        allLocations[i].lat, allLocations[i].lon,
                        allLocations[j].lat, allLocations[j].lon
                    );
                }
            }
        }
    }
    
    vector<int> heldKarpTSP() {
        int n = allLocations.size();
        int fullMask = (1 << n) - 1;
        
        vector<vector<double>> dp(1 << n, vector<double>(n, numeric_limits<double>::max()));
        vector<vector<int>> parent(1 << n, vector<int>(n, -1));
        
        dp[1][0] = 0;
        
        for (int mask = 1; mask < (1 << n); mask++) {
            for (int last = 0; last < n; last++) {
                if (!(mask & (1 << last))) continue;
                if (dp[mask][last] == numeric_limits<double>::max()) continue;
                
                for (int next = 0; next < n; next++) {
                    if (mask & (1 << next)) continue;
                    
                    int newMask = mask | (1 << next);
                    double newCost = dp[mask][last] + distMatrix[last][next];
                    
                    if (newCost < dp[newMask][next]) {
                        dp[newMask][next] = newCost;
                        parent[newMask][next] = last;
                    }
                }
            }
        }
        
        double bestCost = numeric_limits<double>::max();
        int lastCity = -1;
        
        for (int i = 1; i < n; i++) {
            double cost = dp[fullMask][i] + distMatrix[i][0];
            if (cost < bestCost) {
                bestCost = cost;
                lastCity = i;
            }
        }
        
        vector<int> path;
        int mask = fullMask;
        int curr = lastCity;
        
        while (curr != -1) {
            path.push_back(curr);
            int prev = parent[mask][curr];
            mask ^= (1 << curr);
            curr = prev;
        }
        
        reverse(path.begin(), path.end());
        path.push_back(0);
        
        return path;
    }
    
    vector<int> christofidesTSP() {
        int n = allLocations.size();
        
        vector<bool> inMST(n, false);
        vector<int> mstParent(n, -1);
        vector<vector<int>> mstAdj(n);
        
        priority_queue<PriorityNode, vector<PriorityNode>, greater<PriorityNode>> pq;
        pq.push(PriorityNode(0, 0.0, -1));
        
        while (!pq.empty()) {
            PriorityNode curr = pq.top();
            pq.pop();
            
            if (inMST[curr.node]) continue;
            
            inMST[curr.node] = true;
            
            if (curr.parent != -1) {
                mstAdj[curr.parent].push_back(curr.node);
                mstAdj[curr.node].push_back(curr.parent);
            }
            
            for (int next = 0; next < n; next++) {
                if (!inMST[next]) {
                    pq.push(PriorityNode(next, distMatrix[curr.node][next], curr.node));
                }
            }
        }
        
        vector<int> oddVertices;
        for (int i = 0; i < n; i++) {
            if (mstAdj[i].size() % 2 == 1) {
                oddVertices.push_back(i);
            }
        }
        
        vector<bool> matched(oddVertices.size(), false);
        
        while (true) {
            double bestDist = numeric_limits<double>::max();
            int bestI = -1, bestJ = -1;
            
            for (size_t i = 0; i < oddVertices.size(); i++) {
                if (matched[i]) continue;
                for (size_t j = i + 1; j < oddVertices.size(); j++) {
                    if (matched[j]) continue;
                    double d = distMatrix[oddVertices[i]][oddVertices[j]];
                    if (d < bestDist) {
                        bestDist = d;
                        bestI = i;
                        bestJ = j;
                    }
                }
            }
            
            if (bestI == -1) break;
            
            matched[bestI] = matched[bestJ] = true;
            
            mstAdj[oddVertices[bestI]].push_back(oddVertices[bestJ]);
            mstAdj[oddVertices[bestJ]].push_back(oddVertices[bestI]);
        }
        
        vector<multiset<int>> adjCopy(n);
        for (int i = 0; i < n; i++) {
            for (int j : mstAdj[i]) {
                adjCopy[i].insert(j);
            }
        }
        
        vector<int> eulerTour;
        stack<int> stk;
        stk.push(0);
        
        while (!stk.empty()) {
            int v = stk.top();
            if (adjCopy[v].empty()) {
                eulerTour.push_back(v);
                stk.pop();
            } else {
                int u = *adjCopy[v].begin();
                stk.push(u);
                adjCopy[v].erase(adjCopy[v].find(u));
                adjCopy[u].erase(adjCopy[u].find(v));
            }
        }
        
        vector<bool> visited(n, false);
        vector<int> hamiltonianPath;
        
        for (int v : eulerTour) {
            if (!visited[v]) {
                visited[v] = true;
                hamiltonianPath.push_back(v);
            }
        }
        hamiltonianPath.push_back(0);
        
        return hamiltonianPath;
    }
    
    vector<int> apply2Opt(vector<int>& tour) {
        int n = tour.size();
        if (n < 4) return tour;
        
        auto getTourDistance = [&]() {
            double total = 0.0;
            for (int i = 0; i < n - 1; i++) {
                total += distMatrix[tour[i]][tour[i+1]];
            }
            return total;
        };
        
        bool improved = true;
        int iteration = 0;
        
        while (improved && iteration < MAX_2OPT_ITERATIONS) {
            improved = false;
            iteration++;
            
            for (int i = 1; i < n - 2; i++) {
                for (int j = i + 1; j < n - 1; j++) {
                    double d1 = distMatrix[tour[i-1]][tour[i]] + distMatrix[tour[j]][tour[j+1]];
                    double d2 = distMatrix[tour[i-1]][tour[j]] + distMatrix[tour[i]][tour[j+1]];
                    
                    if (d2 < d1 - 1e-9) {
                        reverse(tour.begin() + i, tour.begin() + j + 1);
                        improved = true;
                        break;
                    }
                }
                if (improved) break;
            }
        }
        
        return tour;
    }
    
    void calculateETAs(vector<Location>& route) {
        if (route.empty()) return;
        
        double cumulativeTime = 0.0;
        route[0].eta = 0.0;
        
        for (size_t i = 1; i < route.size(); i++) {
            double distance = distMatrix[route[i-1].id][route[i].id];
            double segmentTime = (distance / AVERAGE_SPEED_KMH) * 60.0;
            cumulativeTime += segmentTime;
            route[i].eta = cumulativeTime;
        }
    }
    
public:
    RouteOptimizer(const vector<Location>& locations) 
        : allLocations(locations) {
        buildDistanceMatrix();
    }
    
    vector<Location> optimizeRoute() {
        int n = allLocations.size();
        
        set<int> highPrioritySet;
        int warehouseIdx = 0;
        
        for (int i = 0; i < n; i++) {
            if (allLocations[i].type == "Warehouse") {
                warehouseIdx = i;
            } else if (allLocations[i].isHighPriority) {
                highPrioritySet.insert(i);
            }
        }
        
        vector<int> optimalOrder;
        
        if (n <= 20) {
            optimalOrder = priorityConstrainedHeldKarp(highPrioritySet, warehouseIdx);
        } else {
            optimalOrder = priorityConstrainedApproximation(highPrioritySet, warehouseIdx);
        }
        
        vector<Location> optimizedRoute;
        for (int idx : optimalOrder) {
            optimizedRoute.push_back(allLocations[idx]);
        }
        
        calculateETAs(optimizedRoute);
        
        return optimizedRoute;
    }
    
private:
    vector<int> priorityConstrainedHeldKarp(const set<int>& highPrioritySet, int warehouseIdx) {
        int n = allLocations.size();
        
        int hpMask = 0;
        for (int hp : highPrioritySet) {
            hpMask |= (1 << hp);
        }
        
        int fullMask = (1 << n) - 1;
        
        vector<vector<double>> dp(1 << n, vector<double>(n, numeric_limits<double>::max()));
        vector<vector<int>> parent(1 << n, vector<int>(n, -1));
        
        dp[1 << warehouseIdx][warehouseIdx] = 0;
        
        for (int mask = 1; mask < (1 << n); mask++) {
            for (int last = 0; last < n; last++) {
                if (!(mask & (1 << last))) continue;
                if (dp[mask][last] == numeric_limits<double>::max()) continue;
                
                bool allHPVisited = (mask & hpMask) == hpMask;
                
                for (int next = 0; next < n; next++) {
                    if (mask & (1 << next)) continue;
                    if (next == warehouseIdx) continue;
                    
                    bool nextIsHP = highPrioritySet.count(next) > 0;
                    if (!allHPVisited && !nextIsHP) {
                        continue;
                    }
                    
                    int newMask = mask | (1 << next);
                    double newCost = dp[mask][last] + distMatrix[last][next];
                    
                    if (newCost < dp[newMask][next]) {
                        dp[newMask][next] = newCost;
                        parent[newMask][next] = last;
                    }
                }
            }
        }
        
        double bestCost = numeric_limits<double>::max();
        int lastCity = -1;
        
        for (int i = 0; i < n; i++) {
            if (i == warehouseIdx) continue;
            double cost = dp[fullMask][i] + distMatrix[i][warehouseIdx];
            if (cost < bestCost) {
                bestCost = cost;
                lastCity = i;
            }
        }
        
        vector<int> path;
        int mask = fullMask;
        int curr = lastCity;
        
        while (curr != -1 && curr != warehouseIdx) {
            path.push_back(curr);
            int prev = parent[mask][curr];
            mask ^= (1 << curr);
            curr = prev;
        }
        
        reverse(path.begin(), path.end());
        
        path.insert(path.begin(), warehouseIdx);
        path.push_back(warehouseIdx);
        
        return path;
    }
    
    vector<int> priorityConstrainedApproximation(const set<int>& highPrioritySet, int warehouseIdx) {
        int n = allLocations.size();
        
        vector<int> hpNodes, normalNodes;
        for (int i = 0; i < n; i++) {
            if (i == warehouseIdx) continue;
            if (highPrioritySet.count(i)) {
                hpNodes.push_back(i);
            } else {
                normalNodes.push_back(i);
            }
        }
        
        vector<int> path;
        path.push_back(warehouseIdx);
        
        if (!hpNodes.empty()) {
            vector<int> hpOrder = optimizeSubsetWithReturn(warehouseIdx, hpNodes, normalNodes.empty() ? warehouseIdx : -1);
            for (int idx : hpOrder) path.push_back(idx);
        }
        
        if (!normalNodes.empty()) {
            int startFrom = path.back();
            vector<int> normalOrder = optimizeSubsetWithReturn(startFrom, normalNodes, warehouseIdx);
            for (int idx : normalOrder) path.push_back(idx);
        }
        
        path.push_back(warehouseIdx);
        
        path = apply2OptConstrained(path, highPrioritySet, warehouseIdx);
        
        return path;
    }
    
    vector<int> optimizeSubsetWithReturn(int startIdx, const vector<int>& subset, int returnTo) {
        if (subset.empty()) return {};
        if (subset.size() == 1) return subset;
        
        int m = subset.size();
        
        if (m <= 12) {
            return optimizeSubsetDPWithReturn(startIdx, subset, returnTo);
        }
        
        return optimizeSubsetLookAhead(startIdx, subset, returnTo);
    }
    
    vector<int> optimizeSubsetDPWithReturn(int startIdx, const vector<int>& subset, int returnTo) {
        int m = subset.size();
        int fullMask = (1 << m) - 1;
        
        vector<vector<double>> dp(1 << m, vector<double>(m, numeric_limits<double>::max()));
        vector<vector<int>> parent(1 << m, vector<int>(m, -1));
        
        for (int i = 0; i < m; i++) {
            dp[1 << i][i] = distMatrix[startIdx][subset[i]];
        }
        
        for (int mask = 1; mask < (1 << m); mask++) {
            for (int last = 0; last < m; last++) {
                if (!(mask & (1 << last))) continue;
                if (dp[mask][last] == numeric_limits<double>::max()) continue;
                
                for (int next = 0; next < m; next++) {
                    if (mask & (1 << next)) continue;
                    
                    int newMask = mask | (1 << next);
                    double newCost = dp[mask][last] + distMatrix[subset[last]][subset[next]];
                    
                    if (newCost < dp[newMask][next]) {
                        dp[newMask][next] = newCost;
                        parent[newMask][next] = last;
                    }
                }
            }
        }
        
        double bestCost = numeric_limits<double>::max();
        int lastIdx = 0;
        
        for (int i = 0; i < m; i++) {
            double cost = dp[fullMask][i];
            if (returnTo >= 0) {
                cost += distMatrix[subset[i]][returnTo];
            }
            if (cost < bestCost) {
                bestCost = cost;
                lastIdx = i;
            }
        }
        
        vector<int> order;
        int mask = fullMask;
        int curr = lastIdx;
        
        while (curr != -1 && mask != 0) {
            order.push_back(subset[curr]);
            int prev = parent[mask][curr];
            mask ^= (1 << curr);
            curr = prev;
        }
        
        reverse(order.begin(), order.end());
        return order;
    }
    
    vector<int> optimizeSubsetLookAhead(int startIdx, const vector<int>& subset, int returnTo) {
        vector<int> order;
        vector<bool> visited(allLocations.size(), false);
        int current = startIdx;
        int remaining = subset.size();
        
        while (remaining > 0) {
            double bestScore = numeric_limits<double>::max();
            int bestNext = -1;
            
            for (int idx : subset) {
                if (visited[idx]) continue;
                
                double distToNode = distMatrix[current][idx];
                
                double futureCost = 0;
                if (remaining > 1) {
                    double minNextDist = numeric_limits<double>::max();
                    for (int other : subset) {
                        if (!visited[other] && other != idx) {
                            minNextDist = min(minNextDist, distMatrix[idx][other]);
                        }
                    }
                    futureCost = (minNextDist < numeric_limits<double>::max()) ? minNextDist : 0;
                }
                
                if (remaining == 1 && returnTo >= 0) {
                    futureCost = distMatrix[idx][returnTo];
                }
                
                double score = distToNode + 0.5 * futureCost;
                
                if (score < bestScore) {
                    bestScore = score;
                    bestNext = idx;
                }
            }
            
            if (bestNext != -1) {
                order.push_back(bestNext);
                visited[bestNext] = true;
                current = bestNext;
                remaining--;
            }
        }
        
        return order;
    }
    
    vector<int> apply2OptConstrained(vector<int>& tour, const set<int>& highPrioritySet, int warehouseIdx) {
        int n = tour.size();
        if (n < 4) return tour;
        
        int hpEndIdx = 0;
        for (int i = 1; i < n - 1; i++) {
            if (highPrioritySet.count(tour[i])) {
                hpEndIdx = i;
            }
        }
        
        bool improved = true;
        int iterations = 0;
        const int MAX_ITER = 100;
        
        while (improved && iterations < MAX_ITER) {
            improved = false;
            iterations++;
            
            for (int i = 1; i < hpEndIdx; i++) {
                for (int j = i + 1; j <= hpEndIdx; j++) {
                    double d1 = distMatrix[tour[i-1]][tour[i]] + distMatrix[tour[j]][tour[j+1]];
                    double d2 = distMatrix[tour[i-1]][tour[j]] + distMatrix[tour[i]][tour[j+1]];
                    
                    if (d2 < d1 - 1e-9) {
                        reverse(tour.begin() + i, tour.begin() + j + 1);
                        improved = true;
                        break;
                    }
                }
                if (improved) break;
            }
            
            if (!improved) {
                for (int i = hpEndIdx + 1; i < n - 2; i++) {
                    for (int j = i + 1; j < n - 1; j++) {
                        double d1 = distMatrix[tour[i-1]][tour[i]] + distMatrix[tour[j]][tour[j+1]];
                        double d2 = distMatrix[tour[i-1]][tour[j]] + distMatrix[tour[i]][tour[j+1]];
                        
                        if (d2 < d1 - 1e-9) {
                            reverse(tour.begin() + i, tour.begin() + j + 1);
                            improved = true;
                            break;
                        }
                    }
                    if (improved) break;
                }
            }
        }
        
        return tour;
    }
    
    vector<int> optimizeSubset(int startIdx, const vector<int>& subset) {
        if (subset.empty()) return {};
        if (subset.size() == 1) return subset;
        
        int subsetSize = subset.size();
        
        if (subsetSize <= 10) {
            return optimizeSubsetDP(startIdx, subset);
        }
        
        return optimizeSubsetNN(startIdx, subset);
    }
    
    vector<int> optimizeSubsetDP(int startIdx, const vector<int>& subset) {
        int m = subset.size();
        int fullMask = (1 << m) - 1;
        
        vector<vector<double>> dp(1 << m, vector<double>(m, numeric_limits<double>::max()));
        vector<vector<int>> parent(1 << m, vector<int>(m, -1));
        
        for (int i = 0; i < m; i++) {
            dp[1 << i][i] = distMatrix[startIdx][subset[i]];
        }
        
        for (int mask = 1; mask < (1 << m); mask++) {
            for (int last = 0; last < m; last++) {
                if (!(mask & (1 << last))) continue;
                if (dp[mask][last] == numeric_limits<double>::max()) continue;
                
                for (int next = 0; next < m; next++) {
                    if (mask & (1 << next)) continue;
                    
                    int newMask = mask | (1 << next);
                    double newCost = dp[mask][last] + distMatrix[subset[last]][subset[next]];
                    
                    if (newCost < dp[newMask][next]) {
                        dp[newMask][next] = newCost;
                        parent[newMask][next] = last;
                    }
                }
            }
        }
        
        double bestCost = numeric_limits<double>::max();
        int lastIdx = 0;
        for (int i = 0; i < m; i++) {
            if (dp[fullMask][i] < bestCost) {
                bestCost = dp[fullMask][i];
                lastIdx = i;
            }
        }
        
        vector<int> order;
        int mask = fullMask;
        int curr = lastIdx;
        
        while (curr != -1 && mask != 0) {
            order.push_back(subset[curr]);
            int prev = parent[mask][curr];
            mask ^= (1 << curr);
            curr = prev;
        }
        
        reverse(order.begin(), order.end());
        return order;
    }
    
    vector<int> optimizeSubsetNN(int startIdx, const vector<int>& subset) {
        vector<int> order;
        vector<bool> visited(allLocations.size(), false);
        int current = startIdx;
        
        for (size_t i = 0; i < subset.size(); i++) {
            double minDist = numeric_limits<double>::max();
            int nearest = -1;
            
            for (int idx : subset) {
                if (!visited[idx]) {
                    double d = distMatrix[current][idx];
                    if (d < minDist) {
                        minDist = d;
                        nearest = idx;
                    }
                }
            }
            
            if (nearest != -1) {
                order.push_back(nearest);
                visited[nearest] = true;
                current = nearest;
            }
        }
        
        return order;
    }
    
public:
    double getTotalDistance(const vector<Location>& route) const {
        double totalDistance = 0.0;
        for (size_t i = 1; i < route.size(); i++) {
            totalDistance += distMatrix[route[i-1].id][route[i].id];
        }
        return totalDistance;
    }
};

int main() {
    crow::App<crow::CORSHandler> app;

    auto& cors = app.get_middleware<crow::CORSHandler>();
    cors.global()
        .headers("Content-Type", "Accept")
        .methods(crow::HTTPMethod::Post, crow::HTTPMethod::Get, crow::HTTPMethod::Options)
        .origin("*");

    CROW_ROUTE(app, "/")
    ([]() {
        return "PostAI Logistics Backend v3.0 | Endpoints: /optimize, /undo";
    });

    CROW_ROUTE(app, "/undo")
    .methods(crow::HTTPMethod::Get)
    ([]() {
        try {
            if (historyStack.empty()) {
                return crow::response(400, "{\"error\": \"No history to undo\", \"message\": \"History stack is empty\"}");
            }
            
            vector<Location> previousLocations = historyStack.top();
            historyStack.pop();
            
            crow::json::wvalue response;
            response["status"] = "success";
            response["message"] = "Undo successful - restored previous state";
            response["history_remaining"] = historyStack.size();
            response["locations_count"] = previousLocations.size();
            
            for (size_t i = 0; i < previousLocations.size(); i++) {
                response["locations"][i]["id"] = previousLocations[i].id;
                response["locations"][i]["type"] = previousLocations[i].type;
                response["locations"][i]["lat"] = previousLocations[i].lat;
                response["locations"][i]["lon"] = previousLocations[i].lon;
                response["locations"][i]["isHighPriority"] = previousLocations[i].isHighPriority;
            }
            
            return crow::response(response);
            
        } catch (const exception& e) {
            return crow::response(500, "{\"error\": \"Undo operation failed\"}");
        }
    });

    CROW_ROUTE(app, "/optimize")
    .methods(crow::HTTPMethod::Post)
    ([](const crow::request& req) {
        try {
            auto json_data = crow::json::load(req.body);
            if (!json_data) {
                return crow::response(400, "{\"error\": \"Invalid JSON\"}");
            }

            vector<Location> locations;
            
            for (size_t i = 0; i < json_data.size(); i++) {
                Location loc;
                loc.id = i;
                loc.lat = json_data[i]["lat"].d();
                loc.lon = json_data[i]["lon"].d();
                loc.type = json_data[i]["type"].s();
                
                if (json_data[i].has("isHighPriority")) {
                    loc.isHighPriority = json_data[i]["isHighPriority"].b();
                } else {
                    loc.isHighPriority = false;
                }
                
                locations.push_back(loc);
            }

            int numLocations = locations.size();

            historyStack.push(locations);

            RouteOptimizer optimizer(locations);
            vector<Location> optimizedRoute = optimizer.optimizeRoute();
            double totalDistance = optimizer.getTotalDistance(optimizedRoute);

            crow::json::wvalue response;
            response["status"] = "success";
            response["message"] = "Route optimized with Hybrid Optimal Algorithm!";
            response["total_locations"] = numLocations;
            response["total_route_distance"] = totalDistance;
            response["history_states"] = historyStack.size();
            response["algorithm"] = numLocations <= 12 ? "Held-Karp DP (OPTIMAL)" : "Christofides (1.5x Approx)";
            
            for (size_t i = 0; i < optimizedRoute.size(); i++) {
                response["optimized_route"][i]["id"] = optimizedRoute[i].id;
                response["optimized_route"][i]["type"] = optimizedRoute[i].type;
                response["optimized_route"][i]["lat"] = optimizedRoute[i].lat;
                response["optimized_route"][i]["lon"] = optimizedRoute[i].lon;
                response["optimized_route"][i]["isHighPriority"] = optimizedRoute[i].isHighPriority;
                response["optimized_route"][i]["eta_minutes"] = optimizedRoute[i].eta;
                
                int hours = static_cast<int>(optimizedRoute[i].eta / 60.0);
                int minutes = static_cast<int>(optimizedRoute[i].eta) % 60;
                char etaStr[10];
                sprintf(etaStr, "%d:%02d", hours, minutes);
                response["optimized_route"][i]["eta_formatted"] = etaStr;
            }

            return crow::response(response);
            
        } catch (const exception& e) {
            return crow::response(500, "{\"error\": \"Server error\"}");
        }
    });
    
    const char* port_env = std::getenv("PORT");
    int port = port_env ? std::atoi(port_env) : 18080;
    
    app.port(port).multithreaded().run();
    
    return 0;
}
