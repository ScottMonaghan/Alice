import board
from  random import random
from adafruit_ht16k33.matrix import Matrix8x8
from time import sleep
 
i2c = board.I2C()
right_eye = Matrix8x8(i2c, auto_write=False)
right_eye.brightness = 7
right_eye.fill(1)
left_eye = Matrix8x8(i2c, address=0x71, auto_write=False)
left_eye.brightness = 7
right_eye.fill(1)

def setMatrixframe(right_eye, left_eye, right_frame, left_frame):
    right_eye.fill(1)
    for y in range(len(right_frame)):
        for x in range(len(right_frame[y])):
            right_eye[y,x] = right_frame[x][y]
    left_eye.fill(1)
    for y in range(len(left_frame)):
        for x in range(len(left_frame[y])):
            left_eye[y,x] = left_frame[x][y]

frame_right_eye_forward = [
    [1,1,1,1,1,1,1,1],
    [1,1,1,1,1,1,1,1],
    [1,1,0,0,0,0,0,1],
    [1,0,0,0,0,0,0,0],
    [1,0,0,0,0,0,0,0],
    [1,0,0,0,0,0,0,0],
    [1,0,0,0,0,0,0,0],
    [1,1,0,0,0,0,0,0]
]
frame_left_eye_forward = [
    [1,1,1,1,1,1,1,1],
    [1,1,1,1,1,1,1,1],
    [1,0,0,0,0,0,1,1],
    [0,0,0,0,0,0,0,1],
    [0,0,0,0,0,0,0,1],
    [0,0,0,0,0,0,0,1],
    [0,0,0,0,0,0,0,1],
    [0,0,0,0,0,0,1,1]
]
frame_right_eye_up = [
    [1,1,0,0,0,0,0,0],
    [1,0,0,0,0,0,0,0],
    [1,0,0,0,0,0,0,0],
    [1,0,0,0,0,0,0,0],
    [1,0,0,0,0,0,0,0],
    [1,1,0,0,0,0,0,1],
    [1,1,1,1,1,1,1,1],
    [1,1,1,1,1,1,1,1]
]
frame_left_eye_up = [
    [0,0,0,0,0,0,1,1],
    [0,0,0,0,0,0,0,1],
    [0,0,0,0,0,0,0,1],
    [0,0,0,0,0,0,0,1],
    [0,0,0,0,0,0,0,1],
    [1,0,0,0,0,0,1,1],
    [1,1,1,1,1,1,1,1],
    [1,1,1,1,1,1,1,1],
]
frame_right_eye_down = [
    [1,1,1,1,1,1,1,1],
    [1,1,1,1,1,1,1,1],
    [1,1,1,1,1,1,1,1],
    [1,1,1,1,1,1,1,1],
    [1,1,0,0,0,0,0,1],
    [1,0,0,0,0,0,0,0],
    [1,0,0,0,0,0,0,0],
    [1,0,0,0,0,0,0,0],
]
frame_left_eye_down = [
    [1,1,1,1,1,1,1,1],
    [1,1,1,1,1,1,1,1],
    [1,1,1,1,1,1,1,1],
    [1,1,1,1,1,1,1,1],
    [1,0,0,0,0,0,1,1],
    [0,0,0,0,0,0,0,1],
    [0,0,0,0,0,0,0,1],
    [0,0,0,0,0,0,0,1],
]

frame_look_right = [
    [1,1,1,1,1,1,1,1],
    [1,1,1,1,1,1,1,1],
    [1,1,1,0,0,0,0,1],
    [1,1,0,0,0,0,0,0],
    [1,1,0,0,0,0,0,0],
    [1,1,0,0,0,0,0,0],
    [1,1,0,0,0,0,0,0],
    [1,1,1,0,0,0,0,0]
]

frame_look_left = [
    [1,1,1,1,1,1,1,1],
    [1,1,1,1,1,1,1,1],
    [1,0,0,0,0,1,1,1],
    [0,0,0,0,0,0,1,1],
    [0,0,0,0,0,0,1,1],
    [0,0,0,0,0,0,1,1],
    [0,0,0,0,0,0,1,1],
    [0,0,0,0,0,1,1,1]
]

frame_look_up_right = [
    [1,1,1,0,0,0,0,0],
    [1,1,0,0,0,0,0,0],
    [1,1,0,0,0,0,0,0],
    [1,1,0,0,0,0,0,0],
    [1,1,0,0,0,0,0,0],
    [1,1,1,0,0,0,0,0],
    [1,1,1,1,1,1,1,1],
    [1,1,1,1,1,1,1,1]
]

frame_look_up_left = [
    [0,0,0,0,0,1,1,1],
    [0,0,0,0,0,0,1,1],
    [0,0,0,0,0,0,1,1],
    [0,0,0,0,0,0,1,1],
    [0,0,0,0,0,0,1,1],
    [0,0,0,0,0,1,1,1],
    [1,1,1,1,1,1,1,1],
    [1,1,1,1,1,1,1,1]
]



frame_blink = [
    [0,0,0,0,0,0,0,0],
    [0,0,0,0,0,0,0,0],
    [0,0,0,0,0,0,0,0],
    [0,0,0,0,0,0,0,0],
    [1,1,1,1,1,1,1,1],
    [0,0,0,0,0,0,0,0],
    [0,0,0,0,0,0,0,0],
    [0,0,0,0,0,0,0,0]
]

current_right_frame = frame_right_eye_forward
current_left_frame = frame_left_eye_forward

def change_direction():
    global current_right_frame
    global current_left_frame
    if current_right_frame != frame_right_eye_forward:
        current_right_frame = frame_right_eye_forward
        current_left_frame = frame_left_eye_forward
    else:
        rnd = int(random() * 6)
        directions = [
            [frame_look_right, frame_look_right]
            ,[frame_look_left, frame_look_left]
            ,[frame_right_eye_up, frame_left_eye_up]
            ,[frame_look_up_left, frame_look_up_left]
            ,[frame_look_up_right, frame_look_up_right]
            ,[frame_right_eye_down, frame_left_eye_down]
        ]
        current_right_frame = directions[rnd][0]        
        current_left_frame = directions[rnd][1]        

while True:
    setMatrixframe(right_eye, left_eye, current_right_frame, current_left_frame)
    right_eye.show()
    left_eye.show()
    sleep(1)
    if random() < 0.5:
        setMatrixframe(right_eye, left_eye, frame_blink,frame_blink)
        left_eye.show()
        right_eye.show()
        sleep(0.1)
    setMatrixframe(right_eye, left_eye, current_right_frame, current_left_frame)
    left_eye.show()
    right_eye.show()
    sleep(1)
    if random() > 0.8:
        change_direction()