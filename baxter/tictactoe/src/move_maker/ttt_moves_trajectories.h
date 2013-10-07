#ifndef TTT_MOVES_TRAJECTORIES_H
#define TTT_MOVES_TRAJECTORIES_H

#include <string>

/**
 * \brief Relation between trajectories and moves for Tic Tac Toe
 * \author Alvaro Castro Gonzalez, acgonzal@ing.uc3m.es
 *
 * Definition of the trajectories required to place a token at any cell of the Tic Tac Toe board
 */

namespace ttt{

std::string cell1x1[3]={"grasp","put@1x1","back@1x1"};
std::string cell1x2[3]={"grasp","put@1x2","back@1x2"};
std::string cell1x3[3]={"grasp","put@1x3","back@1x3"};
std::string cell2x1[3]={"grasp","put@2x1","back@2x1"};
std::string cell2x2[3]={"grasp","put@2x2","back@2x2"};
std::string cell2x3[3]={"grasp","put@2x3","back@2x3"};
std::string cell3x1[3]={"grasp","put@3x1","back@3x1"};
std::string cell3x2[3]={"grasp","put@3x2","back@3x2"};
std::string cell3x3[3]={"grasp","put@3x3","back@3x3"};

}

#endif // TTT_MOVES_TRAJECTORIES_H
