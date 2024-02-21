import time
import Board

print('''
**********************************************************
********Function: Hiwonder Raspberry Pi Expansion Board, Buzzer Control Routine*********
**********************************************************
----------------------------------------------------------
Official website:https://www.hiwonder.com
Online mall:https://hiwonder.tmall.com
----------------------------------------------------------
Tips:
 * Press Ctrl+C to stop running the program. If fail to stop, please try mupltiple time!
----------------------------------------------------------
''')

Board.setBuzzer(0) # close 

Board.setBuzzer(1) # open
time.sleep(0.1) # delay
Board.setBuzzer(0) #close 

time.sleep(1) # delay

Board.setBuzzer(1)
time.sleep(0.5)
Board.setBuzzer(0)