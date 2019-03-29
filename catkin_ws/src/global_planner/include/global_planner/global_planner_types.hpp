namespace Turtlebot
{

template <typename Object>

struct Point
{
    Point(){}
    Point(const Object &x_, const Object &y_) :
        x(x_),
        y(y_)
        {}
    ~Point() = default;
      bool operator==(const Point<Object> &rhs) const
      {
          return (rhs.x == x && rhs.y == y);
      }

      Object x;
      Object y;

};

struct GraphNode
{
    GraphNode(){}
    GraphNode(const Point<double> &cp, const Point<double> &pp, const int &id_, const int &parent_id_,
              const double &g_, const double &cost_) :
        child_point(cp),
        parent_point(pp),
        id(id_),
        parent_id(parent_id_),
        g(g_),
        cost(cost_)
        {}
    ~GraphNode() = default;
    Point<double> child_point;
    Point<double> parent_point;
    int id;
    int parent_id;
    double g;
    double cost;

    bool operator==(const GraphNode &rhs) const
    {
        return (child_point == rhs.child_point) &&
               (parent_point == rhs.parent_point);
    }
    struct CheaperCost
    {
        bool operator()(const GraphNode &lhs, const GraphNode &rhs) const
        {
            return lhs.cost > rhs.cost;
        }
    };

};

}
