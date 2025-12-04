import onnx
import onnxruntime
import numpy as np
import sys

from src.low_level_test import Custom
from unitree_sdk2py.core.channel import ChannelFactoryInitialize

def initialize_low_level_control():

    print("WARNING: Please ensure there are no obstacles around the robot while running this example.")
    input("Press Enter to continue...")

    if len(sys.argv)>1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(0)

    custom = Custom()
    custom.Init()
    custom.Start()
    
    # print(" Hold current position for 2 seconds")
    # time.sleep(2)
    return custom


def main():

    model_checkpoint = "./model_checkpoints/test.onnx"
    
    onnx_model = onnx.load(model_checkpoint)
    onnx.checker.check_model(onnx_model)
    ort_session = onnxruntime.InferenceSession(model_checkpoint, providers=["CPUExecutionProvider"])

    low_level_control = initialize_low_level_control()
    
    while True:

        # get an observation from the robot (placeholder used)
        test_observation = (np.random.uniform(0, 1, size=(1, 1, 37)).astype(np.float32),) # go2
        
        
        # format input for onnx runtime
        onnxruntime_input = {input_arg.name: input_value for input_arg, input_value in zip(ort_session.get_inputs(), test_observation)}
        desired_position = ort_session.run(None, onnxruntime_input)


        # Move the robot to the target position
        print(f"Moving to position: {desired_position}")
        low_level_control.move_to_target(desired_position, 500)  # 500*0.002=1s


if __name__ == "__main__":
    main()
