import argparse
import os
from faceRecognition import FaceDetection
from faceRecognition import FaceRecognition
import cv2

model_path = 'API/models/pretrained/version-RFB-640.pth'
register_path = 'registro.pickle'
h = 480
w = 640
fd = FaceDetection(model_path, h, w)
fr = FaceRecognition()
fr.loadRegister(register_path)

if __name__ == '__main__':

    try:
        parser = argparse.ArgumentParser()
        parser.add_argument('-df', "--dataset_folder", help='Path to register dataset', required=True)
        parser.add_argument('-rf', "--register_file",  help='Register file', required=True)
        args = vars(parser.parse_args())

        dataset_folder = args["dataset_folder"]
        register_filename = args["register_file"]

        files = []
        names = []

        for foldername in os.listdir(dataset_folder):
            for filename in os.listdir(dataset_folder+"/"+foldername):
                files.append(dataset_folder+"/"+foldername+'/'+filename)
                names.append(foldername)

        for i in range(len(files)):
            image = cv2.imread(files[i])
            name = names[i]

            if image is not None:
                image, boxes = fd.detect(image)
                if len(boxes>0):
                    fr.register(image, boxes, name)

        fr.save_register(register_filename)
    except Exception as ex:
        print(ex)
