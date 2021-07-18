import argparse
import logging
import numpy as np
import onnx
import onnxruntime as ort
logging.basicConfig(
    level=logging.INFO,
    format='[%(levelname)s] %(asctime)s.%(msecs)03d : %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S',
)


def parse_args():
    # Parse command line arguments
    parser = argparse.ArgumentParser(
        description='Check model response with ONNX Runtime')
    parser.add_argument('-m', '--model', required=True, help='Model path')
    return parser.parse_args()


def main(args):
    onnx_model = args.model
    model = onnx.load_model(onnx_model)
    onnx.checker.check_model(model)
    graph = onnx.helper.printable_graph(model.graph)
    logging.info(graph)

    ort_session = ort.InferenceSession(onnx_model)
    data = np.array([
        [5, 5, 20],
        [5, 20, 30],
        [30, 10, 50],
    ]).astype(np.float32)
    outputs = ort_session.run(None, {ort_session.get_inputs()[0].name: data})
    logging.info(outputs)


if __name__ == '__main__':
    main(parse_args())
