#! /usr/bin/env python3
from binascii import b2a_base64
from xmlrpc.client import boolean
import rospy
#custom msg called Board_State containing int[255] needed to store board infos
from scrabble.msg import Board_State
from scrabble.msg import New_Word
from std_msgs.msg import Bool
#message of type string needed to tell the game the word inserted by user
import numpy as np
import copy
'''
1)Game starts-> turn to player
2)player picks letter 
3)player choose from GUI if play a word, change rack or pass
4)if a word is composed, camera of world captures the new state in the board
5)camera publish word to the game into a topic after detecting where is 
the new word is and game compute validity of word inserted. 
If word is not valid
loop (message in gui + go to point 3)
publish word to the game to a topic

If word is valid, game send msg to board_state_publisher and the node
update last board state with the new state
6)AI turn-> robot pick up 7 letters and camera look at them and publish
them to the game through a topic

7)game choose best word and publish to a topic exactly position of 
the new word location (or pass if there's no possibility)
8)robot ros node wait msg and then use a ptp motion plan to position letter
from the rack to the board 

9)loop until game stops.

'''

ROW = 17
COL = 17

global board_current_state 
global board_old_state


def print_board(board):
    for i in range(ROW):
        buf = ""
        for j in range(COL):
            if(board.board[i*ROW+j]==0):
                buf+='0 '
            else:
                buf+=chr(board.board[i*ROW+j]) + ' '
        print(buf)        


def update_board(value):
    global board_old_state,board_current_state
    board_old_state = copy.deepcopy(board_current_state)
    print("Vision update")


def retrieve_word_from_board(board,old_board):
    def retrieve_rowcol(x):
        row = x//COL
        col = x%ROW
        return row,col
    #detect orientation of word
    first_letter_pos = -1
    for i in range(len(board.board)):
        if(board.board[i] != 0):
            first_letter_pos=i #0 based
            print(i)
            break
    
    horizontal = True
    h_letters = 0
    v_letters = 0
    r,c = retrieve_rowcol(first_letter_pos)
    #check horizontal
    for i in range(COL):
        #print("H->check: ",str(i + r*COL))
        if(board.board[i + r*COL]!=0):
            h_letters = h_letters + 1
    #check vertical
    for i in range(ROW):
        #print("V->check: ",str(c+i*ROW))
        if(board.board[c+i*ROW]!=0):     
            v_letters = v_letters + 1
    #check validity of move
    if(h_letters > 1 and v_letters > 1):
        print("h: "+str(h_letters)+ " v: "+str(v_letters))
        print("ERROR: TILE CAN BE INSERTED ONLY HORIZONTALLY OR VERTICALLY")
        return
    if(h_letters > 1):
        horizontal = True
    else:
        horizontal = False
    print(horizontal)
    print("Orientation of word: "+ str("horizontal" if horizontal==True else "vertical"))

    #build word inserted
    word = ""
    buffer = ""
    word_passed=False
    last_check=0
    new_word = New_Word()
    if(horizontal):
        new_word.horizontal = True
        for i in range(r*COL, r*COL+COL):
            if (board.board[i]!=0):
                word+=buffer
                word+=chr(board.board[i])
                buffer=""
                word_passed=True
                last_check=i
            elif (old_board.board[i]!=0 and word_passed==False):
                buffer+=chr(old_board.board[i])
            elif (old_board.board[i]!=0 and word_passed==True):
                if(last_check == i-1):
                    word+=chr(old_board.board[i])
                    last_check=i
            elif (old_board.board[i]==0):
                buffer=""
    else:
        new_word.horizontal = False
        for i in range(c,c+ROW*(ROW),ROW):
            if (board.board[i]!=0):
                word+=buffer
                word+=chr(board.board[i])
                buffer=""
                word_passed=True
                last_check=i
            elif (old_board.board[i]!=0 and word_passed==False):
                buffer+=chr(old_board.board[i])
            elif (old_board.board[i]!=0 and word_passed==True):
                if(last_check == i-COL):
                    word+=chr(old_board.board[i])
                    last_check=i
            elif (old_board.board[i]==0):
                buffer=""           
    print("WORD: "+word)
    
    new_word.word = word
    new_word.row = r
    new_word.column = c
    return new_word



def publisher(board_old_state):
    
    difference_array = Board_State()
    for i in range (len(difference_array.board)):
        difference_array.board[i]=0
    while(not rospy.is_shutdown()):
        '''
        print_board(board_current_state) 
        #NICOLA->COMPUTE VISION AND SETTING BOARD STATE INTO 'board_current_state' var
            #(retrieve an array that represents every cell of the board)
        
        #detecting the word inserted and sending it to the game node
        for i in range(len(board_current_state.board)):
            #computing new letters
            difference_array.board[i] = board_current_state.board[i] - board_old_state.board[i]
        print("Difference_array: ")
        print_board(difference_array) 
        new_word = New_Word()
        new_word = retrieve_word_from_board(difference_array,board_old_state)
        print(new_word)
        pub1.publish(board_current_state)
        pub2.publish(new_word)
        print("######################")
        ''' 
        word = New_Word()
        for i in range(len(word.word)):
            word.word[i] = 0
        word.word[0:6]=[ 114,97, 98, 98, 105, 116]
        word.row = 10
        word.column = 13
        print(word)
        pub2.publish(word)
        rospy.sleep(1)
    
    
if __name__ == "__main__":
    rospy.init_node('board_state_publisher', anonymous=True)
    pub1 = rospy.Publisher("scrabble/board_state_publisher", Board_State,queue_size=10)
    pub2 = rospy.Publisher("scrabble/new_word", New_Word,queue_size=0)
    rospy.Subscriber("scrabble/board_state_update_request", Bool, update_board)

    board_old_state = Board_State()
    board_current_state = Board_State ()
    for i in range(len(board_current_state.board)):
        board_current_state.board[i] = 0
        board_old_state.board[i]=0
    publisher(board_current_state)
