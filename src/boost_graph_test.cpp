//
// Created by waxz on 23-4-1.
//

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/visitors.hpp>
#include <boost/graph/depth_first_search.hpp>
#include <boost/graph/dijkstra_shortest_paths_no_color_map.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>

#include "common/clock_time.h"


/*
use graphviz

 sudo apt-get install graphviz
 dot -Tpng test.dot -o test.png
 dot -Tsvg test.dot -o test.svg
 dot test.dot -Tpdf -o test.pdf
 */

struct Vertex {
    float x;
    float y;
    std::string name;
};
struct EdgePair{
    size_t id_start;
    size_t id_end ;
    float dist;
};

struct PathFinding{
    typedef boost::property<boost::edge_weight_t, float> Edge;
    using graph_t = boost::adjacency_list<boost::listS, boost::vecS, boost::directedS, Vertex, Edge>;
    using vertex_t = boost::graph_traits<graph_t>::vertex_descriptor;
    using edge_t = boost::graph_traits<graph_t>::edge_descriptor;

    graph_t g; // construct a graph object

    std::vector<vertex_t> vertex_vec;
    std::vector<float> distances;
    std::vector<vertex_t> pMap;
    std::vector<vertex_t> path;
    std::vector<vertex_t> forward_path;
    bool write_dot = false;


    int createGraph(const std::vector<Vertex>& node_list, std::vector<EdgePair>& edge_list){
        g.clear();

        size_t node_num = node_list.size();
        size_t edge_num = edge_list.size();

        vertex_vec.resize(node_num);
        for(size_t i = 0; i < node_num ;i++){
            vertex_vec[i] =  boost::add_vertex(node_list[i], g);
        }

        for(size_t i = 0; i < edge_num ;i++){
            if(edge_list[i].id_start < node_num
               && edge_list[i].id_end < node_num
               && edge_list[i].dist > 0.0 && edge_list[i].dist < 100.0
            ){
                if(edge_list[i].id_end != edge_list[i].id_start){
                    boost::add_edge(vertex_vec[edge_list[i].id_start], vertex_vec[edge_list[i].id_end], {edge_list[i].dist}, g);
                    std::cout << "add edge: " << edge_list[i].id_start << " to " << edge_list[i].id_end << ", distance " << edge_list[i].dist << "\n";
                }
            }else{
                return -1;
            }
        }



        std::cout << "write_graphviz:\n";
        boost::write_graphviz(std::cout, g, [&](auto &out, auto v) {
                                  out << "[label=\"" << g[v].name << "\"]";
                              },
                              [&](auto &out, auto e) {
                                  out << "[distance=\"" <<
                                  boost::get(boost::edge_weight,g,e) << "\"]";
                              });

        if(write_dot){
            std::ofstream dotfile;
            dotfile.open("a_test_bgl.dot");
            boost::write_graphviz(dotfile, g, [&](auto &out, auto v) {

                                      char buffer[200];
                                      sprintf(buffer,R"([shape=box, style=filled, fillcolor=yellow, label="%s", pose="%.3f,%.3f"])",g[v].name.c_str(),g[v].x, g[v].y );

                                      out << buffer;
                                  },
                                  [&](auto &out, auto e) {
                                      out << "[label=\"" <<
                                          boost::get(boost::edge_weight,g,e) << "\"]";
                                  });
        }

        return 0;
    }

    int findPath(size_t id_start,size_t id_end){
        const size_t numVertices = boost::num_vertices(g);
        size_t node_num = vertex_vec.size();

        if(numVertices < 2){
            return -1;
        }
        if(id_start >= node_num
        &&id_end >= node_num
        && id_start == id_end
        ){
            return -2;
        }

        {


            distances.resize(numVertices);
            pMap.resize(numVertices);

            auto distanceMap = predecessor_map(
                    make_iterator_property_map(pMap.begin(), boost::get(boost::vertex_index, g))).distance_map(
                    make_iterator_property_map(distances.begin(), boost::get(boost::vertex_index, g)));

            auto source = vertex_vec[id_start];
            auto destination =  vertex_vec[id_end];

            boost::dijkstra_shortest_paths(g, source, distanceMap);

            path.clear();
            forward_path.clear();

            vertex_t current = destination;

            if( (pMap[current] != current)){
                while (  (pMap[current] != current) && (current != source) ) {
                    path.push_back(current);
                    current = pMap[current];
                }
            }

            if(current == source ){
                std::cout << "success to find path from " << g[destination].name << " to " << g[source].name   << std::endl;
                path.push_back(source);
            }else{
                std::cout << "fail to find path from " << g[destination].name << " to " << g[source].name   << std::endl;
                return -3;
            }
            std::copy(path.rbegin(), path.rend(), std::back_inserter(forward_path));
            std::cout << "forward_path : \n";
            for (auto &p1: forward_path) {
                std::cout << g[p1].name << ", distance: " << distances[p1] << "\n";
            }
            std::cout << "end of forward_path\n";

            if(write_dot){
                std::ofstream dotfile;
                dotfile.open("a_test_bgl_path.dot");
                boost::write_graphviz(dotfile, g, [&](auto &out, auto v) {

                                          char buffer[200];
                                          auto it = std::find(forward_path.begin(),forward_path.end(), v);
                                          if(it != forward_path.end()){
                                              if(v == source){
                                                  sprintf(buffer,R"([shape=box, style=filled, fillcolor=red, color=green, label="source: %s", pos="%.3f,%.3f"])",g[v].name.c_str(),g[v].x, g[v].y );

                                              }else if(v == destination){
                                                  sprintf(buffer,R"([shape=box, style=filled, fillcolor=red, color=green, label="destination: %s", pos="%.3f,%.3f"])",g[v].name.c_str(),g[v].x, g[v].y );

                                              }else{
                                                  sprintf(buffer,R"([shape=box, style=filled, fillcolor=green, color=green, label="%s", pos="%.3f,%.3f"])",g[v].name.c_str(),g[v].x, g[v].y );

                                              }


                                          } else{
                                              sprintf(buffer,R"([shape=box, style=filled, fillcolor=yellow, label="%s", pose="%.3f,%.3f"])",g[v].name.c_str(),g[v].x, g[v].y );
                                          }

                                          out << buffer;
                                      },
                                      [&](auto &out, auto e) {
                                          out << "[label=\"" <<
                                              boost::get(boost::edge_weight,g,e) << "\"]";
                                      });
            }



        }
        return 0;
    }


};


//https://siavashk.github.io/2019/01/23/boost-graph-library/
void test_1() {



    struct Vertex {
        int value;
        std::string name;
    };
    typedef boost::property<boost::edge_weight_t, float> Edge;
    using graph_t = boost::adjacency_list<boost::listS, boost::vecS, boost::directedS, Vertex, Edge>;
    using vertex_t = boost::graph_traits<graph_t>::vertex_descriptor;
    using edge_t = boost::graph_traits<graph_t>::edge_descriptor;

    struct dfs_visitor : boost::default_dfs_visitor {
        using event_filter = boost::on_tree_edge;
        std::vector<vertex_t> vv;

        void initialize_vertex(vertex_t s, graph_t const &g) const {
            std::cout << "Initialize: " << g[s].value << std::endl;
        }

        void start_vertex(vertex_t s, graph_t const &g) const {
            std::cout << "Start:      " << g[s].value << std::endl;
        }

        void discover_vertex(vertex_t s, graph_t const &g) const {
            std::cout << "Discover:   " << g[s].value << std::endl;
        }

        void finish_vertex(vertex_t s, graph_t const &g) const {
            std::cout << "Finished:   " << g[s].value << std::endl;
        }
    };


    graph_t g; // construct a graph object

    g.clear();

    auto s = boost::add_vertex({10, "s"}, g);
    auto u = boost::add_vertex({11, "u"}, g);
    auto v = boost::add_vertex({22, "v"}, g);
    auto w = boost::add_vertex({33, "w"}, g);
    auto x = boost::add_vertex({44, "x"}, g);
    auto y = boost::add_vertex({55, "y"}, g);


    // Create an edge conecting those two vertices
    boost::add_edge(s, u, {14}, g);
    boost::add_edge(s, v, {26}, g);
    boost::add_edge(s, w, {21}, g);
    boost::add_edge(u, w, {6}, g);
    boost::add_edge(x, y, {1}, g);


    boost::write_graphviz(std::cout, g, [&](auto &out, auto v) {
                              out << "[label=\"" << g[v].value << "\"]";
                          },
                          [&](auto &out, auto e) {
                              out << "[label=\"" << e << "\"]";
                          });


    {
        const int numVertices = num_vertices(g);
        std::vector<int> distances(numVertices);
        std::vector<vertex_t> pMap(numVertices);

        auto distanceMap = predecessor_map(
                make_iterator_property_map(pMap.begin(), boost::get(boost::vertex_index, g))).distance_map(
                make_iterator_property_map(distances.begin(), boost::get(boost::vertex_index, g)));

        auto source = s;
        auto destination = y;

        common::Time t1 = common::FromUnixNow();
        dijkstra_shortest_paths(g, source, distanceMap);
        common::Time t2 = common::FromUnixNow();

        std::vector<vertex_t> path;
        vertex_t current = destination;

        if( (pMap[current] != current)){
            while (  (pMap[current] != current) && (current != source) ) {
                path.push_back(current);
                current = pMap[current];
            }
        }

        if(current == source ){
            std::cout << "success to find path from " << g[destination].name << " to " << g[source].name   << std::endl;
            path.push_back(source);
        }else{
            std::cout << "fail to find path from " << g[destination].name << " to " << g[source].name   << std::endl;
        }
        std::vector<vertex_t> forward_path;
        std::copy(path.rbegin(), path.rend(), std::back_inserter(forward_path));
        common::Time t3 = common::FromUnixNow();

        for (auto &p1: path) {
            std::cout << "path : " << g[p1].name << std::endl;
        }


        for (auto &p1: forward_path) {
            std::cout << "forward_path : " << g[p1].name << ", distance: " << distances[p1] << std::endl;
        }


        std::cout << "use time t2 - t1 " << common::ToMillSeconds(t2 - t1) << " ms\n";

        std::cout << "use time t3 - t1 " << common::ToMillSeconds(t3 - t1) << " ms\n";


    }


}


void test_2(){
    PathFinding pathFinding;
    pathFinding.write_dot = true;


    size_t node_num = 10;
    std::vector<Vertex> node_list(node_num);
    char buffer[100];
    for(size_t i = 0 ; i < node_num; i++){

        sprintf(buffer,"%ld:[%.3f,%.3f]", i, i*0.1, i*0.2);
        node_list[i].name.assign(buffer);

    }

    std::vector<EdgePair> edge_list;
    for(size_t i = 0 ; i < 5; i++){
        edge_list.emplace_back(EdgePair{i,i+1,0.1});
    }
    for(size_t i = 6 ; i < 9; i++){
        edge_list.emplace_back(EdgePair{1,i,0.1});
    }
    edge_list.emplace_back(EdgePair{6,5,0.2});

    pathFinding.createGraph(node_list,edge_list);
    pathFinding.findPath(0,5);


}
int main() {

//    test_1();
    test_2();
}

