/*
Nialan Young, 2022
Bus optimization AI
Version 1.0.0

This program uses reverse mode automatic differentiation
alongside Dijkstra's algorithm to optimize bus routes.
The main function is repsonsible for the input and output.
It also calls the train function which creates examples and trains the program.
To create an example, it calls a function with random inputs.
That function computes Dijkstra's algorithm and its derivative.

*/

#include <iostream>
#include <queue> //This includes vector as well
#include <random>
#include <tuple>
#include <chrono>


using MAIN_TYPE = unsigned short;
constexpr unsigned simulations = 1;
constexpr unsigned calls = 20;
constexpr unsigned iterations = 40;
constexpr unsigned num_examples = 5;

typedef std::pair<MAIN_TYPE,MAIN_TYPE> group; //pair of which node it's connected to, weight
typedef std::tuple<MAIN_TYPE, MAIN_TYPE, char> triplet; //group for bus version; look at logbook

struct bus_stop{
    MAIN_TYPE id;
    MAIN_TYPE num_connections;
    std::vector<bus_stop*> connections;
};

struct bus_route{
    std::vector<MAIN_TYPE> route;
    std::vector<MAIN_TYPE> running_sum;
    bus_route(std::vector<MAIN_TYPE> vertices){
        route = vertices;
        running_sum.push_back(0);
        for(MAIN_TYPE i = 0; i < route.size(); i++){
            running_sum.push_back(running_sum[i] + route[i]);
        }
    }
    inline MAIN_TYPE dist(MAIN_TYPE start, MAIN_TYPE end){
        return running_sum[end] - running_sum[start];
    }
    inline std::vector<MAIN_TYPE> all_dists(MAIN_TYPE start){
        std::vector<MAIN_TYPE> dists;
        for(MAIN_TYPE i = 0; i < route.size(); i++){
            dists.push_back(running_sum[i] - running_sum[start]);
        }
        return dists;
    }
};

namespace map{
    std::vector<std::vector<std::pair<MAIN_TYPE, MAIN_TYPE>>> graph; //vertex weight
    std::vector<std::vector<std::vector<MAIN_TYPE>>> busgraph; //vertex, weight, time
    std::vector<bus_route> busgraph2;
    MAIN_TYPE vertices;
    MAIN_TYPE busvertices;
}

std::vector<std::vector<MAIN_TYPE> > dijkstra(MAIN_TYPE start)
{ //regular version
    std::priority_queue< group, std::vector <group> , std::greater<group> > prq;
    std::vector<std::vector<MAIN_TYPE>> out;
    std::vector<MAIN_TYPE> time(map::vertices, std::numeric_limits<MAIN_TYPE>::max()); //Stores distances
    time[start] = 0;
    prq.push(std::make_pair(0, start));
    while (!prq.empty()) { //repeat until there are no more vertices to check
        MAIN_TYPE i = prq.top().second;
        prq.pop();
  
        for (auto x : map::graph[i]) {
            MAIN_TYPE j = x.first; //these variables aren't really needed, they just make the code look nicer
            MAIN_TYPE weight = x.second;
            if (time[j] > time[i] + weight) { //update the shortest path
                time[j] = time[i] + weight; //update the distance
                out[j] = out[i]; //update the paths vector
                out[j].push_back(i);
                prq.push(std::make_pair(time[j], j)); //add to heap
            }
        }
    }
    return out; //return all the paths
} //O(E log v)

std::vector<std::vector<MAIN_TYPE>> busdijkstra(MAIN_TYPE start, char currtime)
{ //Assumes the bus routes repeat every hour
    std::priority_queue< triplet, std::vector <triplet> , std::greater<triplet> > prq;
    prq.push(std::make_tuple(0, start, currtime));
    std::vector<std::vector<MAIN_TYPE>> out;
    std::vector<MAIN_TYPE> time(map::busvertices, std::numeric_limits<MAIN_TYPE>::max()); //Stores distances
    time[start] = 0;
    std::vector<char> minute; //store what time it is when arriving at a certain node
    while (!prq.empty()) {
        MAIN_TYPE i = std::get<1>(prq.top());
        prq.pop();
  
        for (auto x : map::busgraph[i]) {
            MAIN_TYPE j = x[0];
            MAIN_TYPE weight = x[1];
            char m = (60 + x[2]-minute[i]) % 60; //m is the time waiting at the bus stop
            if (time[j] > time[i] + weight + m) {
                time[j] = time[i] + weight + m;
                out[j] = out[i];
                out[j].push_back(i);
                minute[j] = (minute[i] + weight + m) % 60; //we now have to also update the minute value
                prq.push(std::make_tuple(time[j], j, minute[j]));
            }
        }
    }
    return out;
}

std::vector<MAIN_TYPE> dist_jkstra(MAIN_TYPE start) //this just returns the distance instead
{ //regular version
    std::priority_queue< group, std::vector <group> , std::greater<group> > prq;
    std::vector<MAIN_TYPE> time(map::vertices, std::numeric_limits<MAIN_TYPE>::max()); //Stores distances
    time[start] = 0;
    prq.push(std::make_pair(0, start));
    while (!prq.empty()) {
        MAIN_TYPE i = prq.top().second;
        prq.pop();
  
        for (auto x : map::graph[i]) {
            MAIN_TYPE j = x.first;
            MAIN_TYPE weight = x.second;
            if (time[j] > time[i] + weight) {
                time[j] = time[i] + weight;
                prq.push(std::make_pair(time[j], j));
            }
        }
    }

    return time;
}

std::vector<MAIN_TYPE> busdist_jkstra(MAIN_TYPE start, char currtime)
{ //bus version of the distance returning pathfinding algorithm, times called >= 1000
    std::priority_queue< triplet, std::vector <triplet> , std::greater<triplet> > prq;
    prq.push(std::make_tuple(0, start, currtime));
    std::vector<MAIN_TYPE> time(map::busvertices, std::numeric_limits<MAIN_TYPE>::max()); //Stores distances
    time[start] = 0;
    std::vector<char> minute(map::busgraph.size()); //Shows the minute that the bus arrives at that stop
    while (!prq.empty()) {
        MAIN_TYPE i = std::get<1>(prq.top());
        prq.pop();
  
        for (auto x : map::busgraph[i]) {
            MAIN_TYPE j = x[0];
            MAIN_TYPE weight = x[1];
            char m = (60 + x[2]-minute[i]) % 60;
            if (time[j] > time[i] + weight + m) {
                time[j] = time[i] + weight + m;
                minute[j] = (minute[i] + weight + m) % 60;
                prq.push(std::make_tuple(time[j], j, minute[j]));
            }
        }
    }

    return time;
}

/*std::vector<MAIN_TYPE> busdist_jkstra_2(MAIN_TYPE start, char currtime){
    std::priority_queue< triplet, std::vector <triplet> , std::greater<triplet> > prq;
    prq.push(std::make_tuple(0, start, currtime));
    std::vector<MAIN_TYPE> time(map::busvertices, std::numeric_limits<MAIN_TYPE>::max()); //Stores distances
    time[start] = 0;
    std::vector<char> minute(map::busvertices); //Shows the minute that the bus arrives at that stop
    while (!prq.empty()) {
        MAIN_TYPE i = std::get<1>(prq.top());
        prq.pop();


    }
}*/

void make_edge(MAIN_TYPE start, MAIN_TYPE end, MAIN_TYPE dist){
    map::graph[start].push_back(std::make_pair(end, dist));
    map::graph[end].push_back(std::make_pair(start, dist));
}

void make_bus_route(std::vector<MAIN_TYPE> stops, char start_time){
    MAIN_TYPE temp;
    MAIN_TYPE time = start_time;
    for(int i=0;i<stops.size()-1;++i){
        temp = dist_jkstra(stops[i])[stops[i+1]];
        map::busgraph[stops[i]].push_back({stops[i+1], temp, time});
        time = (time+temp)%60;
        map::busgraph[stops[i+1]].push_back({stops[i], temp, time});
    }
}

int temp_func(MAIN_TYPE start, MAIN_TYPE curr_time){
    std::vector<MAIN_TYPE> dists = busdist_jkstra(start,curr_time);
    int out = 0;
    MAIN_TYPE sizes = 0;
    for(MAIN_TYPE it : dists)
        if(it!=std::numeric_limits<MAIN_TYPE>::max()){
            out += it;
            sizes++;
        }
    return out/sizes;
}

void train( //gradient descent
    double rate = 0.1, //training rate
    unsigned max_iter = iterations, //max iterations
    double epsilon = 0.01, //tolerance
    unsigned ex = num_examples //training examples per training iteration
){
    std::vector<std::vector<std::pair<double,double>>> deriv (map::busgraph.size()); //derivative storage
    double in_tolerance = epsilon+1; //assigning epsilon+1 to pass initial for loop condition
    int size = 1;
    MAIN_TYPE temp1;
    char temp2;
    std::mt19937 gen(std::random_device{}());
    int average = 0;
    int divisor = 0;
    for(int it1 = 0; it1<map::busgraph.size();++it1){ //Average distance
        auto dists = busdist_jkstra(it1,0);
            for(int it2 = 0;it2<map::busgraph.size();++it2){
                if(dists[it2]!=std::numeric_limits<MAIN_TYPE>::max()){
                    average += dists[it2];
                    divisor++;
                }
            }
    }
    average /= divisor;

    for(int k=0; k<max_iter; k++){
        in_tolerance = 0;
        size = 0;
        int ch1, ch2;
        for(int i=0;i<ex;++i){ //create examples within train function
            temp1 = gen()%map::busvertices;
            temp2 = gen()%60;
            if(map::busgraph[temp1].size()!=0)
                for(int i=0;i<map::busgraph.size();++i){
                    deriv[i].resize(map::busgraph[i].size());
                    for(int j=0;j<map::busgraph[i].size();++j){
                        map::busgraph[i][j][0]++;
                        ch1 = temp_func(temp1,temp2);
                        map::busgraph[i][j][0]-=2;
                        ch1 -= temp_func(temp1,temp2);
                        map::busgraph[i][j][0]++; ch1 /= (int)(2*ex); map::busgraph[i][j][2]++;
                        ch2 = temp_func(temp1,temp2);
                        map::busgraph[i][j][2]-=2;
                        ch2 -= temp_func(temp1,temp2);
                        map::busgraph[i][j][2]++; ch2 /= (int)(2*ex);
                        deriv[i][j].first += ch1; deriv[i][j].second += ch2;
                    }
                }
            else {i--;}
        }

        std::vector<std::vector<std::vector<MAIN_TYPE>>> dup = map::busgraph;

        for(int i=0;i<map::busgraph.size();i++){ //change bus graph
            for (int j = 0; j < map::busgraph[i].size(); j++) {
                map::busgraph[i][j][0] -= deriv[i][j].first*rate;
                map::busgraph[i][j][0] %=41;
                map::busgraph[i][j][1] = dist_jkstra(i)[map::busgraph[i][j][0]];
                map::busgraph[i][j][2] -= deriv[i][j].second*rate;
                map::busgraph[i][j][2] %= 60;
                in_tolerance += deriv[i][j].first + deriv[i][j].second;
                size+=2;
            }
        }

        //std::cout << "\titeration " << k << " done\n";
        int avg = 0, tmp = 0;
            for(int it1 = 0; it1<map::busgraph.size();++it1){ //Average distance
                auto dists = busdist_jkstra(it1,0);
                for(int it2 = 0;it2<map::busgraph.size();++it2){
                    if(dists[it2]!=std::numeric_limits<MAIN_TYPE>::max()){
                        avg += dists[it2];
                        tmp++;
                    }
                }
            }
        if(avg/tmp>average){
            map::busgraph = dup;
            k--;
        } else {
            std::cout << avg/tmp << '\n';
            average = avg/tmp;
        }
    }

}

int main() {
map::vertices = 41;
map::graph.resize(41);
make_edge(0,1,482);
make_edge(1,2,704);
make_edge(2,9,684);
make_edge(1,4,445);
make_edge(0,3,512);
make_edge(3,4,279);
make_edge(4,5,203);
make_edge(5,6,198);
make_edge(6,7,259);
make_edge(2,8,681);
make_edge(7,8,177);
make_edge(6,9,191);
make_edge(0,1,197);
make_edge(5,10,189);
make_edge(4,11,154);
make_edge(3,12,194);
make_edge(11,12,196);
make_edge(10,11,188);
make_edge(9,10,335);
make_edge(8,9,192);
make_edge(9,16,188);
make_edge(10,15,200);
make_edge(11,14,190);
make_edge(12,13,211);
make_edge(13,14,190);
make_edge(14,15,194);
make_edge(15,16,202);
make_edge(13,17,199);
make_edge(14,18,203);
make_edge(15,19,191);
make_edge(16,20,204);
make_edge(17,18,207);
make_edge(18,19,195);
make_edge(19,20,355);
make_edge(8,21,511);
make_edge(20,21,172);
make_edge(17,27,195);
make_edge(27,28,193);
make_edge(26,28,186);
make_edge(25,26,195);
make_edge(24,25,198);
make_edge(23,24,609);
make_edge(22,23,270);
make_edge(21,22,198);
make_edge(20,23,191);
make_edge(19,24,184);
make_edge(28,25,184);
make_edge(17,26,77);
make_edge(22,34,196);
make_edge(23,33,200);
make_edge(24,32,194);
make_edge(25,31,191);
make_edge(26,30,189);
make_edge(28,29,208);
make_edge(29,30,195);
make_edge(30,31,190);
make_edge(31,32,192);
make_edge(32,33,695);
make_edge(33,34,108);
make_edge(34,35,185);
make_edge(33,36,185);
make_edge(32,37,187);
make_edge(31,38,185);
make_edge(30,39,193);
make_edge(29,40,760);
make_edge(35,36,204);
make_edge(36,37,198);
make_edge(37,38,177);
make_edge(38,39,195);
std::cout << "map made\n";

map::busvertices = 41;
map::busgraph.resize(41);
make_bus_route({40,36,35,22,23,28,27,20,21,8,9,12,3,0,2},0);
std::cout << "bus route made\n";

float average = 0;
unsigned temp = 0;
for(int it1 = 0; it1<map::busgraph.size();++it1){ //Average distance
    auto dists = busdist_jkstra(it1,0);
    for(int it2 = 0;it2<map::busgraph.size();++it2){
        if(dists[it2]!=std::numeric_limits<MAIN_TYPE>::max()){
            average += dists[it2];
            temp++;
        }
    }
}
std::cout << "Average distance at start: " << average/temp << '\n';
for(int s=0; s<simulations; s++){ //Controls number of simulations

std::cout << "Simulation " << s+1 << '\n';
auto start = std::chrono::high_resolution_clock::now();
for(int i=1;i<=calls;i++){ //* train
    train();
   // std::cout << "training " << i << " done\n";
}
auto stop = std::chrono::high_resolution_clock::now();
auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
std::cout << "Simulation " << s+1 << " took " << duration.count() << " milliseconds\n";
/*for(int i=0;i<map::busgraph.size();i++){
    std::cout << "Starting at node " << i << "\n\n";
    for(auto j : map::busgraph[i]){
        std::cout << "Ends at node " << j[0] <<
        ", with distance " << j[1] <<
        " and leaving at time " << j[2] << '\n';
    }
}*/
average=0; temp=0;
for(int it1 = 0; it1<map::busgraph.size();++it1){ //Average distance
    auto dists = busdist_jkstra(it1,0);
    for(int it2 = 0;it2<map::busgraph.size();++it2){
        if(dists[it2]!=std::numeric_limits<MAIN_TYPE>::max()){
            average += dists[it2];
            temp++;
        }
    }
}
std::cout << "Average distance at end: " << average/temp << '\n' << '\n';
map::busgraph.clear();
map::busgraph.resize(41);
make_bus_route({40,36,35,22,23,28,27,20,21,8,9,12,3,0,2},0);
}
}
