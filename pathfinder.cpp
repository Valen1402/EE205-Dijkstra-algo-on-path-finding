#include <cstring>
#include <cassert>
#include <iostream>
#include "map.h"
#include <queue>
#include <vector>
#include <bits/stdc++.h>
#include <math.h>
#include <sys/time.h>

using std::cout; using std::cerr; using std::endl;
using std::vector; using std::priority_queue;
using std::pair; using std::make_pair; using std::greater;

typedef pair<float, pair<size_t, size_t>> pi;
typedef pair<float, size_t> pai;

void print_usage(const char *prog)
{
    cout << "usage: " << prog << " ALGORITHM: dijkstra" << endl;
}

float Distance(struct map::Map m, size_t cid1, size_t cid2) {
	float x, y;
	x = m.crosses[cid1].x - m.crosses[cid2].x; 
	y = m.crosses[cid1].y - m.crosses[cid2].y;
    return sqrt(x*x + y*y);
   
}

void dijkstra(struct map::Map m, vector<vector<size_t>> AdjList, vector<size_t> &path, struct map::Client client)
{
    vector<size_t>::iterator iv;
	vector<size_t> parent(m.crosses.size(), 0);		// store parent of a crossroad in path	

    size_t src = client.src_cid;
    size_t dst = client.dst_cid;
    
    // initialize pq as min heap
    priority_queue<pi, vector<pi>, greater<pi>> pq;
    pq.push(make_pair(0, make_pair(INT_MAX, src)));

    size_t cid_found;
    float cur_dis;
	
	//dijkstra algo
    while(1){
        cid_found = pq.top().second.second;
        //if that min node is processed, ignore it
        if (parent[cid_found] != 0) { pq.pop(); continue;}
        
        cur_dis = pq.top().first;
        parent[cid_found] = pq.top().second.first;
		pq.pop();
		
		// found the path to dst
		if (cid_found == dst) {
			//cout << "Distance is: "<< cur_dis << endl;
			path.push_back(cid_found);
			while(parent[cid_found] != INT_MAX) {
				cid_found = parent[cid_found];
				path.push_back(cid_found);
			}
			return;
		}
		
		// update all crossroad in adjlist of cid_found
        for (iv = AdjList[cid_found].begin(); iv != AdjList[cid_found].end(); iv++)
        {
            float distance = cur_dis + Distance(m, cid_found, *iv);
            pq.push(make_pair(distance, make_pair(cid_found, *iv)));
        }
    }

};

void astar(struct map::Map m, vector<vector<size_t>> AdjList, vector<size_t> &path, struct map::Client client)
{
	vector<size_t> parent(m.crosses.size(), 0);		// parent of a crossroad in the path
	vector<float> distance(m.crosses.size(), INT_MAX); //from src to crossroads
	vector<float> estimate(m.crosses.size() , 0); // distance to dst in estimate
	vector<float> cost(m.crosses.size(), INT_MAX); // estimate + distance
	vector<bool>  processed(m.crosses.size(), false); // added into cloud: yes/no
    
    size_t src = client.src_cid;
    size_t dst = client.dst_cid;
    
    // initialize pq as min heap
    priority_queue<pai, vector<pai>, greater<pai>> pq;
    
    distance[src] = 0;
    estimate[src] = Distance(m, src, dst);
    pq.push(make_pair (estimate[src], src));

    size_t cid_found;
    float cur_dis;

	// astar algo
    while(1){
        cid_found = pq.top().second;
        //if that min node is processed, ignore it
        if (processed[cid_found] == true) {pq.pop(); continue;}
        processed[cid_found] = true;
        cur_dis = distance[cid_found];
		pq.pop();
		
		// found the path to dst
		if (cid_found == dst) {
			//cout << "Distance is: "<< cur_dis << endl;
			path.push_back(cid_found);
			while(cid_found != src) {
				cid_found = parent[cid_found];
				path.push_back(cid_found);
			}
			return;
		}
		
		// update all crossroad in adjlist of cid_found
        for (vector<size_t>::iterator iv = AdjList[cid_found].begin();
        						iv != AdjList[cid_found].end(); iv++)
        {
        	if(estimate[*iv] == 0) estimate[*iv] = Distance(m, *iv, dst);
        	float dist = cur_dis + Distance(m, cid_found, *iv);
            float new_cost = estimate[*iv] + dist;
            if(new_cost < cost[*iv]) {
            	pq.push(make_pair(new_cost, *iv));
            	cost[*iv] = new_cost;
            	parent[*iv] = cid_found;
            	distance[*iv] = dist;
            }	
        }
    }
}

// to get runtime
///////////////////////////////////////////////////
/*
static void
printElapsedTime(const struct timeval* pstart, const char* str)
{
	struct timeval now;
	double elapsed;

	gettimeofday(&now, NULL);
	elapsed = 1e3 * (now.tv_sec - pstart->tv_sec)
		+ 1e-3 * (now.tv_usec - pstart->tv_usec);
	cout << str << ":\t" << std::setprecision(5) << elapsed << " ms " << endl;
}*/
///////////////////////////////////////////////////


int main(int argc, const char *argv[])
{
    if (argc != 2) {
        print_usage(argv[0]);
        return 1;
    }

	//struct timeval t1;
	
    // Load a map.
    struct map::Map m;
    map::load_map(&m);
    // Shortest paths.
    struct map::Path p;
    // Select algorithm.
    vector<size_t> path;
    
    // construct adjancency list
    vector<vector<size_t>> AdjList(m.crosses.size());
    for(vector<struct map::Road>::iterator ir = m.roads.begin(); ir != m.roads.end(); ir++){
        AdjList[ir->cids[0]].push_back(ir->cids[1]);
        AdjList[ir->cids[1]].push_back(ir->cids[0]);
    }
    
    if (!strncmp(argv[1], "dijkstra", 9)) {
        // Version 1: Use Dijkstra's algorithm.
        //gettimeofday(&t1, NULL);
        
        vector<struct map::Client>::iterator i;
        // call dijkstra for each client, then push their path into p.paths
        for (i = m.clients.begin(); i!= m.clients.end(); i++){
            dijkstra(m, AdjList, path, *i);
            p.paths.push_back(path);
            path.clear();	//path ready for the next client
        }
        //printElapsedTime(&t1, "dijkstra");

    } else if (!strncmp(argv[1], "a-star", 7)) {
        // Version 2: Use A* algorithm.
        //gettimeofday(&t1, NULL);
        
		vector<struct map::Client>::iterator i;
		// call astar for each client, then push their path into p.paths
        for (i = m.clients.begin(); i!= m.clients.end(); i++){
            astar(m, AdjList, path, *i);
            p.paths.push_back(path);
            path.clear();	//path ready for the next client
        }
        //printElapsedTime(&t1, "a-star");

    } else {
        cerr << "ALGORITHM should be either dijkstra or a-star. Given: "
             << argv[1] << "." << endl;
        print_usage(argv[0]);
        return 1;
    }

    // Write results into a file.
    map::store_path(&p);
    return 0;
}
