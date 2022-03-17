#include "viewer.h"

int grid_node_width = 2;
int grid_node_height = 2;

Viewer::Viewer(int width, int height, int edge)
    : width(width), height(height), edge(edge) {
  this->WIN_SIZE[0] = width + 2 * edge;
  this->WIN_SIZE[1] = height + 2 * edge;
}

std::unordered_map<std::string, std::vector<int>> COLORS{
    {"red", std::vector<int>{255, 0, 0}},
    {"green", std::vector<int>{0, 255, 0}},
    {"blue", std::vector<int>{0, 0, 255}},
    {"yellow", std::vector<int>{255, 255, 0}},
    {"grey", std::vector<int>{176, 196, 222}},
    {"purple", std::vector<int>{160, 32, 240}},
    {"black", std::vector<int>{0, 0, 0}},
    {"white", std::vector<int>{255, 255, 255}},
    {"light green", std::vector<int>{204, 255, 229}},
    {"sky blue", std::vector<int>{0, 191, 255}}};

std::unordered_map<std::string, int> COLOR_TO_IDX{
    {"red", 7},    {"green", 1}, {"sky blue", 2},    {"yellow", 3}, {"grey", 4},
    {"purple", 5}, {"black", 6}, {"light green", 0}, {"blue", 8}};

std::vector<std::string> IDX_TO_COLOR{"light green", "green", "sky blue",
                                      "yellow",      "grey",  "purple",
                                      "black",       "red",   "blue"};

// Viewer::draw_ball(std::vector<std::vector<int>> pos_list,
// std::vector<agent_list_t> agent_list)
// {
//     assert(pos_list.size()==agent_list.size());
//     for (size_t i = 0; i < pos_list.size(); i++)
//     {
//         auto t = pos_list[i];
//         auto r = agent_list[i].r;
//         auto color = agent_list[i].color;
//         draw.circle(this->background, COLORS[color], t, r, 0)
//         draw.circle(this->background, COLORS['black'], t, 2, 2)

//     }

// }
