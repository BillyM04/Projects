#include <iostream>
#include <vector>
#include <queue>
#include <limits>

using namespace std;
int da[8][8] = {
    {0,0,0,0,2,0,3,0},
    {0,0,0,0,0,0,1,1},
    {0,0,0,2,0,0,3,0},
    {0,0,2,0,0,0,0,4},
    {2,0,0,0,0,0,0,4},
    {0,0,0,0,0,0,0,0},
    {3,1,3,0,0,0,0,0},
    {0,1,0,4,4,0,0,0}
};

int nodes = 8;
int inf = std::numeric_limits<int>::max();

void obstacle(){
    da[7][4] = 0;
    da[4][7] = 0;
}

void dijkstras(int source, int goal, void (*obstacle)()=nullptr){
//visited and distance array initializing, and creating min-heap
    vector<int> distance(nodes,inf), parent(nodes,-1);
    distance[source] = 0;
    priority_queue<pair<int,int>, vector<pair<int,int>>,greater<pair<int,int>>> pq;
    pq.push({0,source});
    if (obstacle != nullptr) {
        obstacle();
    }
//loop for algorithm
    while(!pq.empty()){
        auto [dist, curr_n] = pq.top();
        pq.pop();

        for(int n = 0; n<nodes; n++){
            if (distance[curr_n] + da[curr_n][n] < distance[n] && da[curr_n][n] > 0){
                distance[n] = distance[curr_n] + da[curr_n][n];
                parent[n] = curr_n;
                pq.push({distance[n],n});
            }
        }
    }
//no path exists
    if(distance[goal] == inf){
        cout<<"no path"<<endl;
        return;
    }
//creating the path
    vector<int> path;
    for(int node = goal; node != -1; node = parent[node]){
        path.push_back(node);
    }
    reverse(path.begin(), path.end());
    for(int i = 0; i < path.size(); i++){
        cout<<path[i]<<", ";
    }
}

int main(){
    vector<int> path = {2,7,0,3,4,2};
    cout<<"shortest path is:"<<endl;
    for(int m = 1; m < path.size(); m++){
        if(path[m] == 5 || path[m-1] == 5){
            cout<<"error";
            break;
        }
        if (m == 4){
            cout<<"uh oh obstacle between 7 and 4!"<<endl;
            dijkstras(path[m-1],path[m],obstacle);
        }
        dijkstras(path[m-1],path[m]);}
}
