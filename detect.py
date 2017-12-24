import cv2
import numpy as np

def isLED(block):
    for i in range(2):
        for j in range(2):
            p = block[i][j]
            if p[0]<255 and p[1]<255 and p[2]<255:
                return False
    return True

def LED_detect(img):
    width, height = img.shape[1], img.shape[0]
    x, y = 0, 0
    for i in range(3, img.shape[0] - 3):
        for j in range(3, img.shape[1] - 3):
            pt = img[i-3:i+3, j-3:j+3, :]
            if isLED(pt):
                y, x = i, j
                break
    if x and y:
        return img[y, x, :].argmax()
    return -1

if __name__ == '__main__':
    img = cv2.imread("g.jpg")
    # 0 green
    BLACK = (0, 0, 0)
    print(img.shape)
    width, height = img.shape[1], img.shape[0]
    x, y = 0, 0
    for i in range(3, img.shape[0] - 3):
        for j in range(3, img.shape[1] - 3):
            pt = img[i-3:i+3, j-3:j+3, :]
            if isLED(pt):
                y, x = i, j
                break
    print(img[y, x, :].argmax())
    cv2.line(img, (x, y-3), (x, y+3), BLACK, 2)
    cv2.line(img, (x-3, y), (x+3, y), BLACK, 2)
    cv2.imshow("histR", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    # print stop