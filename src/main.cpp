#include <iostream>
#include <memory>
#include <queue>
#include <cmath>
#include <fstream>

class Town;
typedef const std::shared_ptr<Town> graph_node_t;

class Node
{
public:
  Node(std::shared_ptr<Node> parent, const int cost, const int estimation, graph_node_t graph_node)
    : parent(parent), cost(cost), estimation(estimation), graph_node(graph_node)
  {
  }

  const std::shared_ptr<Node> parent;
  const int cost;
  const int estimation;
  graph_node_t graph_node;

  class Compare {
  public:
    bool operator() (const std::shared_ptr<Node> lhs, const std::shared_ptr<Node> rhs) const
    {
      return lhs->estimation > rhs->estimation;
    }
  };

  static Node * root_node(const int cost, const int estimation, graph_node_t graph_node)
  {
    return new Node(std::shared_ptr<Node>(nullptr), cost, estimation, graph_node);
  }
};

typedef std::priority_queue<std::shared_ptr<Node>, std::vector<std::shared_ptr<Node>>, Node::Compare> fringe_t;

struct Town
{
  Town(const char name, const int x, const int y)
    : name(name), x(x), y(y)
  {
  }

  static Town * fromStream(std::istream& stream)
  {
    #define stream_read(type, name) type name; { stream >> name; if(stream.eof()) return nullptr; }
    stream_read(char, name)
    stream_read(int, x)
    stream_read(int, y)
    // std::cerr << "Reading town: " << name << " " << x << " " << y << std::endl;
    return new Town(name, x, y);
  }

  const char name;
  const int x;
  const int y;
};

struct Road
{
  explicit Road(const int cost, const char from, const char to)
    : cost(cost), from(from), to(to)
  {
  }

  static Road * fromStream(std::istream& stream)
  {
    #define stream_read(type, name) type name; { stream >> name; if(stream.eof()) return nullptr; }
    stream_read(char, from)
    stream_read(char, to)
    stream_read(int, cost)
    // std::cerr << "Reading road: " << from << " " << to << " " << cost << std::endl;
    return new Road(cost, from, to);
  }

  const int cost;
  const char from;
  const char to;
};

class Map
{
public:
  std::vector<std::shared_ptr<Town>> towns;
  std::vector<std::shared_ptr<Road>> roads;

  const std::shared_ptr<Town> get_town(const char name) const
  {
    for(auto town : towns)
    {
      if(town->name == name)
      {
        return town;
      }
    }
    std::cerr << "No such town: " << name << std::endl;
    exit(3);
    return std::shared_ptr<Town>(nullptr);
  }

  int estimate(const std::shared_ptr<Town> from, const std::shared_ptr<Town> to) const
  {
    #define sqr(a) ((a)*(a))
    return sqrt(sqr(from->x - to->x) + sqr(from->y - to->y));
  }

  int estimate(const char from, const char to) const
  {
    return estimate(get_town(from), get_town(to));
  }

  void read_towns(const std::string& filename)
  {
    std::ifstream file(filename);
    if(file.is_open())
    {
      while(auto town = Town::fromStream(file))
      {
        towns.emplace_back(town);
      }
      file.close();
    }
  }

  void read_roads(const std::string& filename)
  {
    std::ifstream file(filename);
    if(file.is_open())
    {
      while(auto road = Road::fromStream(file))
      {
        roads.emplace_back(road);
      }
      file.close();
    }
  }
};

class AStar
{
public:
  AStar(const Map& map, const std::shared_ptr<Town> to)
    : map(map), to(to)
  {
  }

  const Map& map;
  const std::shared_ptr<Town> to;

  void expand(fringe_t& fringe, const std::shared_ptr<Node> node)
  {
    for(auto road : map.roads)
    {
      // std::cerr << "Current road: " << road->from << " - " << road->to << std::endl;

      if(road->from == node->graph_node->name || road->to == node->graph_node->name)
      {
        const char town_name = (road->from == node->graph_node->name) ? road->to : road->from;
        auto target_town = map.get_town(town_name);
        auto cost = node->cost + road->cost;
        auto estimation = cost + map.estimate(target_town, to);
        fringe.emplace(new Node(node, cost, estimation, target_town));
      }
    }
  }

  const std::shared_ptr<Node> path(fringe_t& fringe)
  {
    while(true)
    {
      std::shared_ptr<Node> node = fringe.top();
      fringe.pop();
      if(node == nullptr)
      {
        std::cerr << "Nullptr?" << std::endl;
        exit(3);
      }

      // std::cerr << "Current node: " << node->graph_node->name << std::endl;

      if(node->graph_node->name == to->name)
      {
        return node;
      }

      expand(fringe, node);
    }
  }

  const std::shared_ptr<Node> path(const std::shared_ptr<Town> from)
  {
    Node::Compare compare;
    fringe_t fringe(compare);

    fringe.emplace(Node::root_node(0, map.estimate(from, to), from));
    return path(fringe);
  }
};

void print_usage(int argc, char const *argv[])
{
  std::cout << "Usage: " << argv[0] << " towns_file roads_file town_from town_to" << std::endl;
}

void check_args(int argc, char const *argv[])
{
  if(argc != 5)
  {
    print_usage(argc, argv);
    exit(1);
  }

  for(int i = 1; i < 5; i++)
  {
    if(argv[i][0] == '\0')
    {
      print_usage(argc, argv);
      exit(1);
    }
  }
}

int main(int argc, char const *argv[])
{
  check_args(argc, argv);

  // Reading command line options
  std::string towns_file(argv[1]);
  std::string roads_file(argv[2]);
  char town_from_name(argv[3][0]);
  char town_to_name(argv[4][0]);


  Map map;
  map.read_towns(towns_file);
  map.read_roads(roads_file);

  auto from = map.get_town(town_from_name);
  auto to = map.get_town(town_to_name);

  AStar astar(map, to);
  auto node = astar.path(from);

  std::cout << "Path cost: " << node->cost << std::endl;

  {
    std::vector<std::shared_ptr<Town>> result;
    while(true)
    {
      if(node == nullptr) { break; }
      result.emplace_back(node->graph_node);
      node = node->parent;
    }

    for(auto town_it = result.crbegin(); town_it != result.crend(); ++town_it)
    {
      std::cout << (*town_it)->name << " ";
    }
  }

  return 0;
}
