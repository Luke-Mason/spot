def main():
    from make_yolo import make_yolov3_model

    yolo_v3 = make_yolov3_model()
    from WeightReader import WeightReader

    weights_path = "../../../../resources/yolov3.weights"

    # Load the weights trained on COCO into the model.
    weight_reader = WeightReader(weights_path)
    weight_reader.load_weights(yolo_v3)

    yolo_v3.save('yolov3.h5')


if __name__ == '__main__':
    main()
