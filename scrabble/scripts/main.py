#! /usr/bin/env python
import os
from turtle import position
import dawg
from string import ascii_uppercase
import string
import random
import rospy
from scrabble.msg import New_Word
from std_msgs.msg import Bool
from scrabble.msg import PutTile
import copy


path="/home/hydran/catkin_ws/src/scrabble/scripts/"
def getboard():
    f =open(path+'board.txt','r')
    stringboard = f.read()
    f.close()
    global boardArray
    boardArray = stringboard.split('\n')

def getboardCopy():
    f =open(path+'board.txt','r')
    stringboard = f.read()
    f.close()
    global boardCopy
    boardCopy = stringboard.split('\n')


def getboardValue():
    f =open(path+'boardValue.txt','r')
    stringboard = f.read()
    f.close()
    global boardValue
    boardValue = stringboard.split('\n')


def getrack():
    f=open(path+'rack.txt','r')
    global stringRack2
    stringRack2 = f.read()
    global rackValues
    rackValues = 	{
	"A":    1,	
	"B":    3,	
    "C":    3,	
	"D":    2,	
    "E":	1,	
	"F":	4,	
	"G":    2,	
    "H":	4,	
	"I":	1,	
	"J":	8,	
	"K":	5,	
	"L":	1,	
	"M":	3,	
	"N":	1,	
	"O":	1,	
	"P":	3,	
	"Q":	10,	
	"R":	1,	
	"S":	1,	
	"T":    1,	
	"U":	1,	
	"V":    4,	
	"W":	4,	
	"X":	8,	
	"Y":	4,	
	"Z":	10	
	}
    f.close()

def laodDist():
    f = open(path+'dictionary.txt','r')
    dictArray = (f.read()).split('\n')
    global completion_dawg
    print('here')
    completion_dawg = dawg.CompletionDAWG(dictArray)
    f.close()

def crossCheck(char,row,x):
    nword = char
    k = row+1
    row-=1

    while row>=0 and boardArray[row][x] != '#':
        nword = boardArray[row][x] + nword
        row-=1

    while k<17 and boardArray[k][x] != '#':
        nword =  nword + boardArray[k][x] 
        k+=1

    if nword in completion_dawg:
        return True
    else :
        return False


def checkWord(word,row,x,col,rack,id):
    if x == 17 and ((word in completion_dawg) == False):
        return

    global possArray
    global possStart

    if x == 17:
        if len(rack) < 7 and boardArray[row][x-len(word)-1]=='#':
            possArray.append(word)
            if id == 1:
                possStart.append([row,x-len(word),id])
            else:
                possStart.append([x-len(word),row,id])
        return

    if x > col and word in completion_dawg and x<17  and boardArray[row][x]=='#' and len(rack) < 7:
        if boardArray[row][x-len(word)-1]=='#':
            possArray.append(word)
            if id == 1:
                possStart.append([row,x-len(word),id])
            else:
                possStart.append([x-len(word),row,id])

    if len(rack) ==0 :
        return

    if boardArray[row][x] == '#':
        for i,char in enumerate(rack):
            word = word + char
            if completion_dawg.has_keys_with_prefix(word) and crossCheck(char,row,x):
                rack = rack[:i] + rack[i+1:]
                checkWord(word,row,x+1,col,rack,id)
                rack = rack[:i]+char+rack[i:]
            word = word[:len(word)-1]
    else:
        word = word + boardArray[row][x]
        checkWord(word,row,x+1,col,rack,id)
        
      
def findPlace(id):
    for i,row in enumerate(boardArray):
        for j,element in enumerate(row):
            if element!='#' and boardArray[i][j-1]=='#' :
                for x in range(max(j-7,0),min(j,16)):
                    checkWord("",i,x,j,stringRack2,id)
                    if i+1<17 and boardArray[i+1][j]=='#':
                        checkWord("",i+1,x,j,stringRack2,id)
                    if i-1>=0 and boardArray[i-1][j]=='#' :
                        checkWord("",i-1,x,j,stringRack2,id)

def crossValue(row,col,id):        
    cost = 0
    if id == 1:
        k = row+1
        row-=1
        while row>=0 and boardArray[row][col] != '#':
            cost = cost + rackValues[boardArray[row][col]]
            row-=1

        while k<17 and boardArray[k][col] != '#':
            cost =  cost + rackValues[boardArray[k][col]] 
            k+=1

    if id == 2:
        k = col+1
        col-=1
        while col>=0 and boardArray[row][col] != '#':
            cost = cost + rackValues[boardArray[row][col]]
            col-=1

        while k<17 and boardArray[row][k] != '#':
            cost =  cost + rackValues[boardArray[row][k]] 
            k+=1
    return cost


def costFunc(strr,i,j,id):
    cost = 0
    dw = 0
    tw = 0
    cw = 0
    if id == 1:
        for col in range(j,j+len(strr)):
            if boardValue[i][col] == "1" or boardValue[i][col] == "2" or boardValue[i][col] == "3" :
                cost += int(rackValues[strr[col-j]]) * int(boardValue[i][col])
            else:
                cost += int(rackValues[strr[col-j]])
            if boardValue[i][col] == "4":
                dw+=1
            if boardValue[i][col] == "5":
                tw+=1
            cw = cw + crossValue(i,col,id)
        cost = cost * (2 ** dw)
        cost = cost * (3 ** tw)
        cost = cost + cw

    if id == 2:
        for row in range(i,i+len(strr)):
            if boardValue[row][j] == "1" or boardValue[row][j] == "2" or boardValue[row][j] == "3" :
                cost += int(rackValues[strr[row-i]]) * int(boardValue[row][j])
            else:
                cost += int(rackValues[strr[row-i]])
            if boardValue[row][j] == 4:
                dw+=1
            if boardValue[row][j] == 5:
                tw+=1
            cw = cw + crossValue(row,j,id)

        cost = cost * (2 ** dw)
        cost = cost * (3 ** tw)
        cost = cost + cw
    return cost

def move():
    getboard()
    global stringRack2
    global cRack
    getrack()
    stringRack2 = cRack
    getboardValue()
    
    global possArray
    possArray = []
    global possStart
    possStart = []

    laodDist()
    global boardArray
    if boardArray[7][7] == '#':
        for x in range(0,7):
            checkWord("",7,x,7,stringRack2,1)
    else :
        findPlace(1)
        boardArray = [*zip(*boardArray)]
        findPlace(2)
        
    mx = -1
    ansIndex = -1
    for i,strr in enumerate(possArray):
        if costFunc(strr,possStart[i][0],possStart[i][1],possStart[i][2]) > mx:
            mx = costFunc(strr,possStart[i][0],possStart[i][1],possStart[i][2])
            ansIndex = i
    
    if ansIndex == -1:
        global passcnt 
        if passcnt == 1:
            print("END GAME")
            exit(0)
        else:
            print("Changing Rack ")
            changeRack(0,[])
        return
    getboardCopy()
    
    if(possStart[ansIndex][2]==1):
        print(" ---- HORIZONTLY ---- ")
        f = open(path+'board.txt','w')
        for i,strr in enumerate(boardCopy):
            for j,char in enumerate(strr):
                if i==possStart[ansIndex][0] and j >= possStart[ansIndex][1] and j<possStart[ansIndex][1]+len(possArray[ansIndex]):
                    f.write(possArray[ansIndex][j-possStart[ansIndex][1]])
                else :
                    f.write(strr[j])
            if i != len(boardCopy)-1: 
                f.write('\n')

    if(possStart[ansIndex][2]==2):
        print(" ---- VERTICALLY---- ")
        f = open(path+'board.txt','w')
        for i,strr in enumerate(boardCopy):
            for j,char in enumerate(strr):
                if j==possStart[ansIndex][1] and i >= possStart[ansIndex][0] and i<possStart[ansIndex][0]+len(possArray[ansIndex]):
                    f.write(possArray[ansIndex][i-possStart[ansIndex][0]])
                else :
                    f.write(strr[j])
            if i != len(boardCopy)-1: 
                f.write('\n')

    print("Best Posible String is " + possArray[ansIndex] + ". Starting is at row "+ str(possStart[ansIndex][0]+1)+" and col at " + str(possStart[ansIndex][1]+1)+" and point is "+str(mx))
    
    global cscore 
    cscore += mx
    print(possArray[ansIndex])
    if(possArray[ansIndex]==""):
        changeRack(0)
    #logica per non inserire lettere  già presenti
    return possArray[ansIndex],possStart[ansIndex][0]+1,possStart[ansIndex][1]+1,(possStart[ansIndex][2]==1)

def userMove():
    getboard()
    global boardArray
    getboardCopy()
    new_word = New_Word()
    new_word = rospy.wait_for_message("/scrabble/new_word",New_Word)
    word = ""
    for i in range(len(new_word.word)):
        if(new_word.word[i]!=0):
            word+=chr(new_word.word[i])
    '''
    d = input("Enter 1 for horizontal word\nEnter 2 for vertical word\n>")
    id = int(d)
    if id != 1 and id != 2:
        print("select proper choice\n")
        return False
    word = input('enter your word\n>')
    '''
    
    word = word.upper()
    print("WORD:"+word)
    print(f"in row: {new_word.row} and column: {new_word.column}")
    if not word in completion_dawg:
        print("Word does not exist\n")
        return False
    ro = new_word.row#input('Enter row of your word starting point\n>')
    co = new_word.column#input('Enter column of your word starting point\n>')
    row = int(ro)
    col = int(co)
    x=row-1
    y=col-1
    if (new_word.horizontal == True):
        id =1
    else:
        id =2 
    nword=word
    global boardArray
    global userRack

    if id ==1:
        for i in range(0,len(word)-1):
            if not crossCheck(word[i],x,y+i):
                print("Cross Check is not valid\n")
                return False

            #j = userRack.find(word[i])
            if boardArray[x][y+i]!='#' and boardArray[x][y+i] != word[i]:
                print("Wrong Placed word entered \n")
                return False
            '''
            elif boardArray[x][y+i]=='#':
                if j == -1:
                    print("letter "+word[i]+" does not exist in rack \n")
                    return False
                elif j!=-1:
                    userRack = userRack[0:j] + userRack[j+1:]
            '''

    if id ==2:
        boardArray = [*zip(*boardArray)]
        for i in range(0,len(word)-1):
            if not crossCheck(word[i],y,x+i):
                print("Cross Check is not valid\n")
                return False
            else:
                print("Valid cross check")

        boardArray = [*zip(*boardArray)]
        for i in range(0,len(word)-1):    
            #j = userRack.find(word[i])
            if boardArray[x+i][y]!='#' and boardArray[x+i][y] != word[i]:
                print("Wrong Placed word entered \n")
                return False
            '''
            elif boardArray[x+i][y]=='#':
                if j == -1:
                    print("letter "+word[i]+" does not exist in rack \n")
                    return False
                elif j!=-1:
                    userRack = userRack[0:j] + userRack[j+1:]
            '''
    
    global userScore 
    userScore += costFunc(word,row-1,col-1,id)

    getboardCopy()
    
    if(id==1):
        print(" ---- HORIZONTALY ---- \nWord is :"+word+"\n")
        f = open(path+'board.txt','w')
        for i,strr in enumerate(boardCopy):
            for j,char in enumerate(strr):
                if i==row-1 and j >= col-1 and j<col-1+len(word):
                    f.write(word[j-col+1])
                else :
                    f.write(strr[j])
            if i != len(boardCopy)-1: 
                f.write('\n')
        
    if(id==2):
        print(" ---- VERTICALLY---- \nWord is :"+word+"\n")
        f = open(path+'board.txt','w')
        for i,strr in enumerate(boardCopy):
            for j,char in enumerate(strr):
                if j==col-1 and i >= row-1 and i<row-1+len(word):
                    f.write(word[i-row+1])
                else :
                    f.write(strr[j])
            if i != len(boardCopy)-1: 
                f.write('\n')

    pub1.publish(True)
    #Send msg to board_state_publisher to save current state of board




def changeRack(which,positions,rack=""):
    if which%2==0:
        global cRack
        print("RACK:_ "+rack+"_")
        if(rack==""):
            for i in range(0,2):
                rack+=random.choice(['A','E','I','O','U'])
            for i in range(0,5):
                rack+=random.choice(string.ascii_uppercase)
        else:
            for i in range(0,7):
                if(rack[i]==' '):
                    rack = rack.replace(' ',random.choice(string.ascii_uppercase),1)
        cRack=copy.deepcopy(rack)
            
    '''
    else:
        global userRack
        userRack = ""
        for i in range(0,2):
            userRack += random.choice(['A','E','I','O','U'])
        for i in range(2,7):
            userRack += random.choice(string.ascii_uppercase)
    '''

def passUser():
    global passcnt
    passcnt = 1

if __name__ == "__main__":
    word_pub = rospy.Publisher('/scrabble/put_tile_on_board_command', PutTile, queue_size=10)
    rospy.init_node('game_node', anonymous=True)
    global pub1
    pub1 = rospy.Publisher("scrabble/board_state_update_request",Bool,queue_size=1)
    f = open(path+'board.txt','w')
    for i in range(0,17):
        if i==16:
            f.write("#################")
        else:
           f.write("#################\n")
    f.close()
    print("\n----------- ARE YOU READY TO LOSE ? ------------\n\n\n")
    userName = input('Please Enter Your Name: ')
    movecnt = 0
    global cscore
    cscore = 0
    global userSore
    global passcnt
    passcnt=""
    userScore = 0
    global cRack
    global userRack
    changeRack(0,positions=[])
    changeRack(1,positions=[])
    while 1:
        print('\n---Current Board----\n')
        #os.system('python printBoard.py')
        os.system('rosrun scrabble printBoard.py')
        print("\nComputer Score : " + str(cscore) +"\t\t\t"+str(userName)+" Score :"+str(userScore)+"\n")
        #print("Computer's Rack: "+cRack+"\t"+userName+"'s Rack: " + userRack+"\n")
        print("Computer's Rack: "+cRack+"\n")
        if movecnt%2==0:
            word,row,col,mode = move()
            rack_pos=[0,0,0,0,0,0,0]
            rrack= copy.deepcopy(cRack)
            command = PutTile()
            letterno=1
            i=0
            for x in word:
                rack_counter=1         
                for y in rrack:
                    if (y== x):
                        rrack=rrack.replace(x,' ',1)
                        print(str(rack_counter)+" "+str(x)+"->"+str(rrack))
                        rack_pos[i]=rack_counter
                        print(rack_pos[i])
                        rack_counter+=1
                        break
                    rack_counter+=1
                i+=1
                letterno+=1
            
            command.rack_pos = copy.deepcopy(rack_pos)
            print(command.rack_pos) 
            command.target_row[0] = row
            command.target_col[0] = col
            i=0
            for x in word: 
                #horizontal
                if(mode==0):
                        if(command.rack_pos[i]!=0):
                            command.target_row[i] = row+i
                            command.target_col[i] = col
                #vertical
                else:
                        if(command.rack_pos[i]!=0):
                            command.target_col[i] = col+i
                            command.target_row[i] = row   
                i+=1
            print(command) 
            input("Press Enter to continue...")
            word_pub.publish(command)
            
            changeRack(movecnt,command.rack_pos,rrack)
            movecnt += 1
        else:
            userIn = input("1.) To Place Word  2.) To Change Rack  3.) To Pass  4.) To Quit\n>")
            #global passcnt
            if userIn == '1':
                if userMove() == False:
                    continue
                #changeRack(movecnt)
                passcnt = 0
                movecnt += 1
            elif userIn == '2':
                #changeRack(movecnt)
                passcnt = 0
                movecnt += 1
            elif userIn=='3':
                passUser()
                movecnt += 1
            elif userIn=='4':
                print("SEE!!!! Told You")
                exit(0)
            else :
                print("Please select valid choice")
