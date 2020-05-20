import numpy as np
import cv2
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import math
import time



def convert_hls(image):
    return cv2.cvtColor(image, cv2.COLOR_RGB2HLS)


def convert_gray_scale(image):
    return cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)


def apply_smoothing(image, kernel_size=5):
    return cv2.GaussianBlur(image, (kernel_size, kernel_size), 0)


def detect_edges(image, low_threshold=150, high_threshold=200):
    return cv2.Canny(image, low_threshold, high_threshold)

def WarpPerspecitve(img):
    img_size = (img.shape[1], img.shape[0])

    src = np.float32(
        [[685, 446],
         [1090, 710],
         [220, 710],
         [595, 446]])

    dst = np.float32(

        [[900, 0],
         [900, 710],
         [250, 710],
         [250, 0]])

    M = cv2.getPerspectiveTransform(src, dst)
    Minv = cv2.getPerspectiveTransform(dst, src)

    binary_warped = cv2.warpPerspective(img, M, img_size, flags=cv2.INTER_LINEAR)

    return binary_warped

def histogram(binary_warped):
    histogram = np.sum(binary_warped[binary_warped.shape[0] // 2:, :], axis=0)

    return histogram

def filter_region(image, vertices):
    mask = np.zeros_like(image)
    if len(mask.shape) == 2:
        cv2.fillPoly(mask, vertices, 255)
    else:
        cv2.fillPoly(mask, vertices, (255,) * mask.shape[2])
    return cv2.bitwise_and(image, mask)


def select_region(image):
    bl_x=0
    bl_y=0.9
    br_x=0.15
    br_y=1
    tl_x=0.3
    tl_y=0.65
    tr_x=0.35
    tr_y=0.65
    rows, cols = image.shape[:2]
    bottom_left = [cols*bl_x, rows*bl_y]
    top_left    = [cols*tl_x, rows*tl_y]
    bottom_right= [cols*br_x, rows*br_y]
    top_right   = [cols*tr_x, rows*tr_y]

    bottom_left1 = [cols * (1-bl_x), rows * bl_y]
    top_left1 = [cols * (1-tl_x), rows * tl_y]
    bottom_right1 = [cols * (1-br_x), rows * br_y]
    top_right1 = [cols * (1-tr_x), rows * tr_y]


    vertices_l = np.array([[bottom_left, top_left, top_right, bottom_right]], dtype=np.int32)
    vertices_r = np.array([[bottom_left1, top_left1, top_right1, bottom_right1]], dtype=np.int32)
    a=filter_region(image, vertices_l)
    b=filter_region(image, vertices_r)
    result=a+b
    return result


def hough_lines(image):
    return cv2.HoughLinesP(image, rho=1, theta=np.pi / 180, threshold=60, minLineLength=60, maxLineGap=200)


def draw_lines(img, lines, color=[255, 0, 0], thickness=3):
    line_img = np.zeros(
        (
            img.shape[0],
            img.shape[1],
            3
        ),
        dtype=np.uint8
    )
    img = np.copy(img)
    if lines is None:
        return

    for line in lines:
        for x1, y1, x2, y2 in line:
            cv2.line(line_img, (x1, y1), (x2, y2), color, thickness)

    img = cv2.addWeighted(img, 0.8, line_img, 1.0, 0.0)

    return img


def select_white_yellow(image):
    converted = convert_hls(image)
    # white color mask
    lower = np.uint8([0, 200, 0])
    upper = np.uint8([255, 255, 255])
#    lower = np.uint8([0,210,0])
#    upper = np.uint8([255,255,255])
    white_mask = cv2.inRange(converted, lower, upper)

    # yellow color mask
    lower = np.uint8([10, 0, 100])
    upper = np.uint8([255, 255, 255])
#    lower = np.uint8([40,20,100])
#    upper = np.uint8([255,255,255])
    yellow_mask = cv2.inRange(converted, lower, upper)

    # combine the mask
    mask = cv2.bitwise_or(white_mask, yellow_mask)

    return cv2.bitwise_and(image, image, mask=mask)

def lane_mid(image, lines):
    global prev_left_line_x, prev_right_line_x
    left_line_x = []
    left_line_y = []
    right_line_x = []
    right_line_y = []
    inaccuracy_count=0
    reliability=0
    going_pixels=[0,0]
    y_pixel=500
    img = np.copy(image)

    for line in lines:
        x1, y1, x2, y2 = line[0]
        cv2.line(img, (x1, y1), (x2, y2), (0, 255, 0), 3)
        if (x2 - x1) == 0:
            slope = 100
        else:
            slope = float(y2 - y1) / float(x2 - x1)

        if abs(math.fabs(slope)) < 0.4:
            inaccuracy_count+=1
            continue
        if ((x1 >= float(image.shape[1] / 2)) and slope>=0) :
            right_line_x.extend([x1, x2])
            right_line_y.extend([y1, y2])
        elif ((x1 < float(image.shape[1] / 2)) and 0>=slope):
            left_line_x.extend([x1, x2])
            left_line_y.extend([y1, y2])

    ########## left max, right min##############
    print(inaccuracy_count ,len(lines))
    min_y = img.shape[0] * 0.5
    max_y = img.shape[0]

#    print('left_line_x: ', left_line_x, 'left_line_y: ', left_line_y)
#    print('right_line_x: ', right_line_x, 'right_line_y: ', right_line_y)
    if (inaccuracy_count>=5 or len(lines)<=2):
        going_pixels=[0,0]
        reliability=0

    elif (len(left_line_x)!=0 and len(right_line_x)!=0):
        poly_left = np.poly1d(np.polyfit(
            left_line_y,
            left_line_x,
            deg=1
        ))

        left_x_start = int(poly_left(max_y))
        left_x_end = int(poly_left(min_y))

        poly_right = np.poly1d(np.polyfit(
            right_line_y,
            right_line_x,
            deg=1
        ))

        right_x_start = int(poly_right(max_y))
        right_x_end = int(poly_right(min_y))


        img = draw_lines(
            img,
            [[
                [left_x_start, max_y, left_x_end, int(min_y)],
                [right_x_start, max_y, right_x_end, int(min_y)],
                ]],
                [0,0,255],
                3
            )

        left_x=poly_left[1]*y_pixel+poly_left[0]
        right_x=poly_right[1]*y_pixel+poly_right[0]
        going_pixels = [(left_x+right_x)/2,y_pixel]
        reliability=1

    elif(len(right_line_x)!=0):
        poly_right = np.poly1d(np.polyfit(
            right_line_y,
            right_line_x,
            deg=1
        ))

        right_x_start = int(poly_right(max_y))
        right_x_end = int(poly_right(min_y))


        img = draw_lines(
            img,
            [[
                [right_x_start, max_y, right_x_end, int(min_y)],
                ]],
                [0,0,255],
                3
            )
        right_x=poly_right[1]*y_pixel+poly_right[0]
        going_pixels = [(right_x-300),y_pixel]
        reliability=1

    elif (len(left_line_x)!=0):
        poly_left = np.poly1d(np.polyfit(
            left_line_y,
            left_line_x,
            deg=1
        ))

        left_x_start = int(poly_left(max_y))
        left_x_end = int(poly_left(min_y))

        img = draw_lines(
            img,
            [[
                [left_x_start, max_y, left_x_end, int(min_y)],
                ]],
                [0,0,255],
                3
            )

        left_x=poly_left[1]*y_pixel+poly_left[0]
        going_pixels = [(left_x+300),y_pixel]
        reliability=1


    #return img, going_pixels
    return img, going_pixels, reliability



def find_delta(image_shape, going_pixels, line_image, fps):

############################# angle1 ###################################
    #print(going_pixels[0]-(image_shape[1]/2))
    angle_radian = math.atan(((going_pixels[0]-(image_shape[1]/2))/2)/(image_shape[0]-going_pixels[1])) ## /2
    angle_degree = angle_radian * (180/np.pi)

    #print('mid x1', mid_x1, 'mid x2', mid_x2, 'y1', y1, 'y2', y2)
    #print('radian1: ',angle_radian, 'degree1', angle_degree)

#############################  pid1 ###################################

    fontType = cv2.FONT_HERSHEY_SIMPLEX
    p = 0.05



    degree = "left" if angle_degree < 0 else "right"
    degree_text = str(round(abs(angle_degree), 3)) + '. ' + degree
    cv2.putText(line_image, degree_text, (30, 100), fontType, 1., (255, 255, 255), 3)

########################### distance ######################################

    direction = "left" if going_pixels[0]-(image_shape[1]/2) < 0 else "right"
    deviation_text = 'target is ' + str(round(abs(going_pixels[0]-(image_shape[1]/2)), 3)) + 'pixel ' + direction + ' of center'
    cv2.putText(line_image, deviation_text, (30, 150), fontType, 1., (255, 255, 255), 3)

########################## FPS ####################################################

    deviation_text = str(round(fps, 3)) + ' FPS'
    cv2.putText(line_image, deviation_text, (30, 50), fontType, 1., (255, 255, 255), 3)

    return line_image, angle_degree




cap = cv2.VideoCapture("contest3.mp4")

prevTime = 0
image_shape=[0,0]
prev_left_line_x=0
prev_right_line_x=0
while(cap.isOpened()):
    ret, frame = cap.read()
    image_shape=[frame.shape[0],frame.shape[1]]
    curTime = time.time()
    sec = curTime - prevTime
    prevTime = curTime

    fps = 1/(sec)





    white_yellow_image = select_white_yellow(frame)

    gray_image = convert_gray_scale(white_yellow_image)

    blurred_image = apply_smoothing(gray_image)

    edge_image = detect_edges(blurred_image)

    masked_image = select_region(edge_image)

    Perspective_lines = WarpPerspecitve(masked_image)

    hough_line = hough_lines(masked_image)

    #print(type(hough_line))
    if(type(hough_line).__module__ == np.__name__):
        lane_detection, deviation_pixels, reliability = lane_mid(frame, hough_line)

        list_of_lines = hough_lines(masked_image)

        #frame_warped = WarpPerspecitve(frame)

        #line_image = draw_lines(frame_warped, list_of_lines)
        if(reliability==1):
            final_image, image_delta=find_delta(image_shape, deviation_pixels, lane_detection, fps)
            cv2.imshow('frame2', final_image)
        else:
            cv2.imshow('frame2', lane_detection)


    else:
        cv2.imshow('frame2', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()


cv2.imshow('lane_detection', lane_detection)
cv2.waitKey(0)
cv2.destroyAllWindows()
