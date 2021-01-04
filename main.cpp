#include <bits/stdc++.h>

using namespace std;

#define ROW 9
#define COL 10

typedef pair<int, int> Pair;

typedef pair<double, pair<int, int>> pPair;

struct Cell {
    int parent_i; // 0 <= i <= ROW-1
    int parent_j; // 0 <= j <= COL-1
    double f, g, h; // f = g + h
};

// check whether given cell is valid or not
bool is_valid(int row, int col) {
    // return true if row num and col num is in range
    return (row >= 0) && (row < ROW) &&
           (col >= 0) && (col < COL);
}

// check whether given cell is blocked or not
bool is_unblocked(int grid[][COL], int row, int col) {
    if (grid[row][col] == 1) {
        return true;
    } else {
        return false;
    }
}

// check whether destination cell has been reached or not
bool is_destination(int row, int col, Pair dest) {
    if (row == dest.first && col == dest.second) {
        return true;
    } else {
        return false;
    }
}

// calculate the 'h' heuristics
double calculate_h_value(int row, int col, Pair dest) {
    // return distance to goal
    return ((double)sqrt((row - dest.first)*(row - dest.first)
                         + (col - dest.second)*(col - dest.second)));
}

// trace the path from the source to destination
void trace_path(Cell cell_details[][COL], Pair dest) {
    printf("\nThe Path is ");
    int row = dest.first;
    int col = dest.second;

    stack<Pair> Path;

    while (!(cell_details[row][col].parent_i == row
             && cell_details[row][col].parent_j == col)) {
        Path.push(make_pair(row, col));
        int tmp_row = cell_details[row][col].parent_i;
        int tmp_col = cell_details[row][col].parent_j;
        row = tmp_row;
        col = tmp_col;
    }

    Path.push(make_pair(row, col));
    while (!Path.empty()) {
        pair<int, int> p = Path.top();
        Path.pop();
        printf("-> (%d, %d) ", p.first, p.second);
    }
}

// find the shortest path between
// given source cell to destination cell
// according to A* search algorithm
void a_star_search(int grid[][COL], Pair src, Pair dest) {
    // source is out of range
    if (!is_valid(src.first, src.second)) {
        printf("Source is invalid\n");
        return;
    }

    // destination is out of range
    if (!is_valid(dest.first, dest.second)) {
        printf("Destination is invalid\n");
        return;
    }

    // source or destination is blocked
    if (!is_unblocked(grid, src.first, src.second) ||
        !is_unblocked(grid, dest.first, dest.second)) {
        printf("Source or destination is blocked\n");
        return;
    }

    // destination cell is same as source cell
    if (is_destination(src.first, src.second, dest)) {
        printf("We are already at the destination\n");
        return;
    }

    // create closed list and initialize it to false
    // which means that no cell has been included yet
    // closed list is implemented as boolean 2D array
    bool closed_list[ROW][COL];
    memset(closed_list, false, sizeof(closed_list));

    // details of cell
    Cell cell_details[ROW][COL];

    int i, j;
    for (i = 0; i < ROW; ++i) {
        for (j = 0; j < COL; ++j) {
            cell_details[i][j].f = FLT_MAX;
            cell_details[i][j].g = FLT_MAX;
            cell_details[i][j].h = FLT_MAX;
            cell_details[i][j].parent_i = -1;
            cell_details[i][j].parent_j = -1;
        }
    }

    // initialize start node
    i = src.first, j = src.second;
    cell_details[i][j].f = 0.0;
    cell_details[i][j].g = 0.0;
    cell_details[i][j].h = 0.0;
    cell_details[i][j].parent_i = i;
    cell_details[i][j].parent_j = j;

    // create open list <f, <i, j>>, f = g + h
    set<pPair> open_list;

    // put starting cell on open list
    // set its f as 0
    open_list.insert(make_pair(0.0, make_pair(i, j)));

    bool found_dest = false;

    while (!open_list.empty()) {
        pPair p = *open_list.begin();
        open_list.erase(open_list.begin());

        // add this vertex to closest list
        i = p.second.first;
        j = p.second.second;
        closed_list[i][j] = true;

        double g_new, h_new, f_new;

        // 1st successor (North)
        if (is_valid(i - 1, j)) {
            // destination cell is same as
            // current successor
            if (is_destination(i - 1, j, dest)) {
                // set parent of destination cell
                cell_details[i - 1][j].parent_i = i;
                cell_details[i - 1][j].parent_j = j;
                printf("The destination cell is found\n");
                trace_path(cell_details, dest);
                found_dest = true;
                return;
            }
                // successor is not closed and unblocked
            else if (!closed_list[i - 1][j] &&
                     is_unblocked(grid, i - 1, j)) {
                g_new = cell_details[i][j].g + 1.0;
                h_new = calculate_h_value(i - 1, j, dest);
                f_new = g_new + h_new;

                // it isn't on open list, add it to open list
                // it is on open list already, select lower f
                if (cell_details[i - 1][j].f == FLT_MAX ||
                    cell_details[i - 1][j].f > f_new)
                {
                    open_list.insert(make_pair(f_new,make_pair(i - 1, j)));

                    // update detail of this cell
                    cell_details[i - 1][j].f = f_new;
                    cell_details[i - 1][j].g = g_new;
                    cell_details[i - 1][j].h = h_new;
                    cell_details[i - 1][j].parent_i = i;
                    cell_details[i - 1][j].parent_j = j;
                }
            }
        }

        // 2nd successor (South)
        if (is_valid(i + 1, j)) {
            // destination cell is same as
            // current successor
            if (is_destination(i + 1, j, dest)) {
                // set parent of destination cell
                cell_details[i + 1][j].parent_i = i;
                cell_details[i + 1][j].parent_j = j;
                printf("The destination cell is found\n");
                trace_path(cell_details, dest);
                found_dest = true;
                return;
            }
                // successor is not closed and unblocked
            else if (!closed_list[i + 1][j] &&
                     is_unblocked(grid, i + 1, j)) {
                g_new = cell_details[i][j].g + 1.0;
                h_new = calculate_h_value(i + 1, j, dest);
                f_new = g_new + h_new;

                // it isn't on open list, add it to open list
                // it is on open list already, select lower f
                if (cell_details[i + 1][j].f == FLT_MAX ||
                    cell_details[i + 1][j].f > f_new)
                {
                    open_list.insert(make_pair(f_new, make_pair(i + 1, j)));

                    // update detail of this cell
                    cell_details[i + 1][j].f = f_new;
                    cell_details[i + 1][j].g = g_new;
                    cell_details[i + 1][j].h = h_new;
                    cell_details[i + 1][j].parent_i = i;
                    cell_details[i + 1][j].parent_j = j;
                }
            }
        }

        // 3rd successor (East)
        if (is_valid(i, j + 1)) {
            // destination cell is same as
            // current successor
            if (is_destination(i, j + 1, dest)) {
                // set parent of destination cell
                cell_details[i][j + 1].parent_i = i;
                cell_details[i][j + 1].parent_j = j;
                printf("The destination cell is found\n");
                trace_path(cell_details, dest);
                found_dest = true;
                return;
            }
                // successor is not closed and unblocked
            else if (!closed_list[i][j + 1] &&
                     is_unblocked(grid, i, j + 1)) {
                g_new = cell_details[i][j].g + 1.0;
                h_new = calculate_h_value(i, j + 1, dest);
                f_new = g_new + h_new;

                // it isn't on open list, add it to open list
                // it is on open list already, select lower f
                if (cell_details[i][j + 1].f == FLT_MAX ||
                    cell_details[i][j + 1].f > f_new)
                {
                    open_list.insert(make_pair(f_new, make_pair(i, j + 1)));

                    // update detail of this cell
                    cell_details[i][j + 1].f = f_new;
                    cell_details[i][j + 1].g = g_new;
                    cell_details[i][j + 1].h = h_new;
                    cell_details[i][j + 1].parent_i = i;
                    cell_details[i][j + 1].parent_j = j;
                }
            }
        }

        // 4th successor (West)
        if (is_valid(i, j - 1)) {
            // destination cell is same as
            // current successor
            if (is_destination(i, j - 1, dest)) {
                // set parent of destination cell
                cell_details[i][j - 1].parent_i = i;
                cell_details[i][j - 1].parent_j = j;
                printf("The destination cell is found\n");
                trace_path(cell_details, dest);
                found_dest = true;
                return;
            }
                // successor is not closed and unblocked
            else if (!closed_list[i][j - 1] &&
                     is_unblocked(grid, i, j - 1)) {
                g_new = cell_details[i][j].g + 1.0;
                h_new = calculate_h_value(i, j - 1, dest);
                f_new = g_new + h_new;

                // it isn't on open list, add it to open list
                // it is on open list already, select lower f
                if (cell_details[i][j - 1].f == FLT_MAX ||
                    cell_details[i][j - 1].f > f_new)
                {
                    open_list.insert(make_pair(f_new, make_pair(i, j - 1)));

                    // update detail of this cell
                    cell_details[i][j - 1].f = f_new;
                    cell_details[i][j - 1].g = g_new;
                    cell_details[i][j - 1].h = h_new;
                    cell_details[i][j - 1].parent_i = i;
                    cell_details[i][j - 1].parent_j = j;
                }
            }
        }

        // 5th successor (North-East)
        if (is_valid(i - 1, j + 1)) {
            // destination cell is same as
            // current successor
            if (is_destination(i - 1, j + 1, dest)) {
                // set parent of destination cell
                cell_details[i - 1][j + 1].parent_i = i;
                cell_details[i - 1][j + 1].parent_j = j;
                printf("The destination cell is found\n");
                trace_path(cell_details, dest);
                found_dest = true;
                return;
            }
                // successor is not closed and unblocked
            else if (!closed_list[i - 1][j + 1] &&
                     is_unblocked(grid, i - 1, j + 1)) {
                g_new = cell_details[i][j].g + 1.414;
                h_new = calculate_h_value(i - 1, j + 1, dest);
                f_new = g_new + h_new;

                // it isn't on open list, add it to open list
                // it is on open list already, select lower f
                if (cell_details[i - 1][j + 1].f == FLT_MAX ||
                    cell_details[i - 1][j + 1].f > f_new)
                {
                    open_list.insert(make_pair(f_new, make_pair(i - 1, j + 1)));

                    // update detail of this cell
                    cell_details[i - 1][j + 1].f = f_new;
                    cell_details[i - 1][j + 1].g = g_new;
                    cell_details[i - 1][j + 1].h = h_new;
                    cell_details[i - 1][j + 1].parent_i = i;
                    cell_details[i - 1][j + 1].parent_j = j;
                }
            }
        }

        // 6th successor (North-West)
        if (is_valid(i - 1, j - 1)) {
            // destination cell is same as
            // current successor
            if (is_destination(i - 1, j - 1, dest)) {
                // set parent of destination cell
                cell_details[i - 1][j - 1].parent_i = i;
                cell_details[i - 1][j - 1].parent_j = j;
                printf("The destination cell is found\n");
                trace_path(cell_details, dest);
                found_dest = true;
                return;
            }
                // successor is not closed and unblocked
            else if (!closed_list[i - 1][j - 1] &&
                     is_unblocked(grid, i - 1, j - 1)) {
                g_new = cell_details[i][j].g + 1.414;
                h_new = calculate_h_value(i - 1, j - 1, dest);
                f_new = g_new + h_new;

                // it isn't on open list, add it to open list
                // it is on open list already, select lower f
                if (cell_details[i - 1][j - 1].f == FLT_MAX ||
                    cell_details[i - 1][j - 1].f > f_new)
                {
                    open_list.insert(make_pair(f_new, make_pair(i - 1, j - 1)));

                    // update detail of this cell
                    cell_details[i - 1][j - 1].f = f_new;
                    cell_details[i - 1][j - 1].g = g_new;
                    cell_details[i - 1][j - 1].h = h_new;
                    cell_details[i - 1][j - 1].parent_i = i;
                    cell_details[i - 1][j - 1].parent_j = j;
                }
            }
        }

        // 7th successor (South-East)
        if (is_valid(i + 1, j + 1)) {
            // destination cell is same as
            // current successor
            if (is_destination(i + 1, j + 1, dest)) {
                // set parent of destination cell
                cell_details[i + 1][j + 1].parent_i = i;
                cell_details[i + 1][j + 1].parent_j = j;
                printf("The destination cell is found\n");
                trace_path(cell_details, dest);
                found_dest = true;
                return;
            }
                // successor is not closed and unblocked
            else if (!closed_list[i + 1][j + 1] &&
                     is_unblocked(grid, i + 1, j + 1)) {
                g_new = cell_details[i][j].g + 1.414;
                h_new = calculate_h_value(i + 1, j + 1, dest);
                f_new = g_new + h_new;

                // it isn't on open list, add it to open list
                // it is on open list already, select lower f
                if (cell_details[i + 1][j + 1].f == FLT_MAX ||
                    cell_details[i + 1][j + 1].f > f_new)
                {
                    open_list.insert(make_pair(f_new, make_pair(i + 1, j + 1)));

                    // update detail of this cell
                    cell_details[i + 1][j + 1].f = f_new;
                    cell_details[i + 1][j + 1].g = g_new;
                    cell_details[i + 1][j + 1].h = h_new;
                    cell_details[i + 1][j + 1].parent_i = i;
                    cell_details[i + 1][j + 1].parent_j = j;
                }
            }
        }

        // 8th successor (South-West)
        if (is_valid(i + 1, j - 1)) {
            // destination cell is same as
            // current successor
            if (is_destination(i + 1, j - 1, dest)) {
                // set parent of destination cell
                cell_details[i + 1][j - 1].parent_i = i;
                cell_details[i + 1][j - 1].parent_j = j;
                printf("The destination cell is found\n");
                trace_path(cell_details, dest);
                found_dest = true;
                return;
            }
                // successor is not closed and unblocked
            else if (!closed_list[i + 1][j - 1] &&
                     is_unblocked(grid, i + 1, j - 1)) {
                g_new = cell_details[i][j].g + 1.0;
                h_new = calculate_h_value(i + 1, j - 1, dest);
                f_new = g_new + h_new;

                // it isn't on open list, add it to open list
                // it is on open list already, select lower f
                if (cell_details[i + 1][j - 1].f == FLT_MAX ||
                    cell_details[i + 1][j - 1].f > f_new)
                {
                    open_list.insert(make_pair(f_new, make_pair(i + 1, j - 1)));

                    // update detail of this cell
                    cell_details[i + 1][j - 1].f = f_new;
                    cell_details[i + 1][j - 1].g = g_new;
                    cell_details[i + 1][j - 1].h = h_new;
                    cell_details[i + 1][j - 1].parent_i = i;
                    cell_details[i + 1][j - 1].parent_j = j;
                }
            }
        }
    }

    // when destination cell is not found
    // and open list is empty, failed to reach
    // destination cell
    if (!found_dest) {
        printf("Failed to find the destination cell\n");
    }
}

int main() {
    // 1 --> cell is not blocked
    // 0 --> cell is blocked
    int grid[ROW][COL] = {
            {1, 0, 1, 1, 1, 1, 0, 1, 1, 1},
            {1, 1, 1, 0, 1, 1, 1, 0, 1, 1},
            {1, 1, 1, 0, 1, 1, 0, 1, 0, 1},
            {0, 0, 1, 0, 1, 0, 0, 0, 0, 1},
            {1, 1, 1, 0, 1, 1, 1, 0, 1, 0},
            {1, 0, 1, 1, 1, 1, 0, 1, 0, 0},
            {1, 0, 0, 0, 0, 1, 0, 0, 0, 1},
            {1, 0, 1, 1, 1, 1, 0, 1, 1, 1},
            {1, 1, 1, 0, 0, 0, 1, 0, 0, 1}
    };

    // source is the left-most bottom-most corner
    Pair src = make_pair(8, 0);

    // destination is the left-most top-most corner
    Pair dest = make_pair(0, 9);

    a_star_search(grid, src, dest);

    return 0;
}
