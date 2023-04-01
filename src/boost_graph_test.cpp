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

//https://siavashk.github.io/2019/01/23/boost-graph-library/
void test_1() {

    using namespace boost;


    struct Vertex {
        int value;
        std::string name;
    };
    typedef property<edge_weight_t, float> Edge;
    using graph_t = adjacency_list<listS, vecS, directedS, Vertex, Edge>;
    using vertex_t = graph_traits<graph_t>::vertex_descriptor;
    using edge_t = graph_traits<graph_t>::edge_descriptor;

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
                make_iterator_property_map(pMap.begin(), get(vertex_index, g))).distance_map(
                make_iterator_property_map(distances.begin(), get(vertex_index, g)));

        auto source = s;
        auto destination = w;

        common::Time t1 = common::FromUnixNow();
        dijkstra_shortest_paths(g, source, distanceMap);
        common::Time t2 = common::FromUnixNow();

        std::vector<vertex_t> path;
        vertex_t current = destination;
        while (current != source) {
            path.push_back(current);
            current = pMap[current];
        }
        path.push_back(source);
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

int main() {

    test_1();

}

