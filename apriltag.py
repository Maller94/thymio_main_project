import pupil_apriltags
import cv2 as cv2

def locateAprilTag(image,choice = 1):
    img = cv2.imread(image,0)
    at_detector = pupil_apriltags.Detector()
    result = at_detector.detect(img)

    if result != []:
        if choice == 0:
            return f'Tag_id: {result[0].tag_id}'
        elif choice == 1:
            return result[0].tag_id
        elif choice == 2:
            return result
    else:
        print('Could not locate apriltag')
        return result

if __name__ == "__main__":
    print(locateAprilTag('aprilTag.png',2))