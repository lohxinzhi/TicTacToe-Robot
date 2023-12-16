
#board w input as matrix
board = [["-", "-", "-"], 
         ["-", "-", "-"], 
         ["-", "-", "-"]]
player = 'O'
computer = 'X'
redoPlayer = False
compDone = False
playerDone = False

#---------------pre-defined functions-----------------
def printBoard(board):
    print(board[0][0] + " | " + board[0][1] + " | " + board[0][2])
    print("---------")
    print(board[1][0] + " | " + board[1][1] +  " | " + board[1][2])
    print("---------")
    print(board[2][0] + " | " + board[2][1] + " | " + board[2][2])
    print("\n")

def spaceIsFree(row_pos, col_pos):
    if board[row_pos][col_pos] == '-':
        return True 
    return False 

def insertLetter(board, letter, row_pos, col_pos):
    global redoPlayer
    if not redoPlayer:
        if spaceIsFree(row_pos, col_pos):
            board[row_pos][col_pos] = letter 
            printBoard(board)
            if checkDraw(board):
                print("Draw!")
                # exit() 
            if checkWin(board):
                if letter == 'X':
                    print("Bot wins!")
                    # exit()
                else:
                    print("Player wins!")
                    # exit()
            return 
        else:
            print("invalid, filled position")
            row_pos, col_pos = (input("enter your coordinates: ").split(","))
            row_pos = int(row_pos) - 1
            col_pos = int(col_pos) - 1
            insertLetter(board, letter, row_pos, col_pos)
            return    

def checkWin(board):
    #row
    if (board[0][0] == board[0][1] == board[0][2] and board[0][0] != '-'):
        return True
    elif (board[1][0] == board[1][1] == board[1][2] and board[1][0] != '-'):
        return True
    elif (board[2][0] == board[2][1] == board[2][2] and board[2][0] != '-'):
        return True
    #col
    elif (board[0][0] == board[1][0] == board[2][0] and board[0][0] != '-'):
        return True
    elif (board[0][1] == board[1][1] == board[2][1] and board[0][1] != '-'):
        return True
    elif (board[0][2] == board[1][2] == board[2][2] and board[0][2] != '-'):
        return True
    #diagonal
    elif (board[0][0] == board[1][1] == board[2][2] and board[0][0] != '-'):
        return True
    elif (board[0][2] == board[1][1] == board[2][0] and board[0][2] != '-'):
        return True
    else:
        return False
    
def checkWhichMarkWon(board, mark):
    #row
    if (board[0][0] == board[0][1] == board[0][2] and board[0][0] == mark):
        return True
    elif (board[1][0] == board[1][1] == board[1][2] and board[1][0] == mark):
        return True
    elif (board[2][0] == board[2][1] == board[2][2] and board[2][0] == mark):
        return True
    #col
    elif (board[0][0] == board[1][0] == board[2][0] and board[0][0] == mark):
        return True
    elif (board[0][1] == board[1][1] == board[2][1] and board[0][1] == mark):
        return True
    elif (board[0][2] == board[1][2] == board[2][2] and board[0][2] == mark):
        return True
    #diagonal
    elif (board[0][0] == board[1][1] == board[2][2] and board[0][0] == mark):
        return True
    elif (board[0][2] == board[1][1] == board[2][0] and board[0][2] == mark):
        return True
    else:
        return False

def checkDraw(board):
    for i in range(0,3):
        for j in range(0,3):
            if board[i][j] == "-":
                return False
    return True 
7
#---------------coordinate on grid---------------------
def gridcoordinate(row_coor, col_coor):
    x_coor = 0
    y_coor = 0
    if row_coor == 0 and col_coor == 0:
        x_coor,y_coor = 214,60

    elif row_coor == 0 and col_coor == 1:
        x_coor,y_coor = 214,0

    elif row_coor == 0 and col_coor == 2:
        x_coor,y_coor = 214,-60

    elif row_coor == 1 and col_coor == 0:
        x_coor,y_coor = 174,60

    elif row_coor == 1 and col_coor == 1:
        x_coor,y_coor = 174,0

    elif row_coor == 1 and col_coor == 2:
        x_coor,y_coor= 174,-60

    elif row_coor == 2 and col_coor == 0:
        x_coor,y_coor = 114,60

    elif row_coor == 2 and col_coor == 1:
        x_coor,y_coor = 114,0
        
    elif row_coor == 2 and col_coor == 2:
        x_coor,y_coor = 114,-60

    return x_coor, y_coor

#--------------comp movement---------------------------
def compMove(board):
    bestScore = -800
    best_comp_row = 0
    best_comp_col = 0
    if not redoPlayer:
        for i_comp in range(0,3):
            for j_comp in range(0,3):
                if board[i_comp][j_comp] == "-":
                    board[i_comp][j_comp] = computer
                    score = minimax(board, False)
                    board[i_comp][j_comp] = "-"
                    if score > bestScore:
                        bestScore = score 
                        best_comp_row = i_comp
                        best_comp_col = j_comp
        insertLetter(board, computer,best_comp_row,best_comp_col)
    return best_comp_row, best_comp_col

def minimax(board, isMaximizing):
    if checkWhichMarkWon(board, computer):
        return 1 
    elif checkWhichMarkWon(board, player):
        return -1 
    elif checkDraw(board):
        return 0
        
    if isMaximizing:
        bestScore = -800 
        for i in range(0,3):
            for j in range(0,3):
                if board[i][j] == "-":
                    board[i][j] = computer 
                    score = minimax(board, False)
                    board[i][j] = '-'
                    if score > bestScore:
                        bestScore = score
        return bestScore 
    else:
        bestScore = 800 
        for i in range(0,3):
            for j in range(0,3):
                if board[i][j] == '-':
                    board[i][j] = player 
                    score = minimax(board, True)
                    board[i][j] = '-'
                    if score < bestScore:
                        bestScore = score 
        return bestScore
#-----------------start of process---------------------
#playerDone = True --> to set true when ahmad makes his move
#once player is done, computer makes its move

def main():
    row_coor = 0
    col_coor = 0
    playerDone = True # triger robot decision
    if playerDone == True:
        scanned_grid = [2,1,2,1,1,0,0,0,0]
        #converting into grid
        grid_matrix = [[scanned_grid[0], scanned_grid[1], scanned_grid[2]],
            [scanned_grid[3], scanned_grid[4], scanned_grid[5]],
            [scanned_grid[6], scanned_grid[7], scanned_grid[8]]]

        for i in range(0,3):
            for j in range(0,3):
                if grid_matrix[i][j] == 2: #computer
                    board[i][j] = "X"
                elif grid_matrix[i][j] == 1: #human
                    board[i][j] = "O"
                else:
                    board[i][j] = "-"

        row_coor, col_coor = compMove(board) 
        print(grid_matrix)
        print(row_coor, col_coor)
        x_coor, y_coor = gridcoordinate(row_coor, col_coor)
        print(x_coor, y_coor)
        playerDone = False

if __name__ == '__main__':
    main() 
        








