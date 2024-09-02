import rclpy
from rclpy.node import Node
from yolov8_msgs.msg import DetectionArray  # yolov8_msgs 패키지에서 메시지 가져오기
import RPi.GPIO as GPIO

class PersonDetectionNode(Node):
    def __init__(self):
        super().__init__('person_detection_node')
        
        # DetectionArray 토픽을 구독합니다.
        self.subscription = self.create_subscription(
            DetectionArray,
            '/yolo/detections',  # YOLOv8 탐지 결과 토픽
            self.detection_callback,
            10  # QoS 설정
        )
        self.subscription  # prevent unused variable warning
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(3, GPIO.OUT, inital=GPIO.HIGH)

    def detection_callback(self, msg):
        # 메시지 내의 각 탐지 결과에 대해 반복합니다.
        for detection in msg.detections:
            # 사람의 class_id는 일반적으로 0입니다.
            if detection.class_id == 0 and detection.class_name == "person":
                self.get_logger().info(f"Person detected with score: {detection.score}")
                print(f"Person detected at position: ({detection.bbox.center.position.x}, {detection.bbox.center.position.y})")
                GPIO.output(3, GPIO.HIGH)
            else:
                GPIO.output(3,GPIO.LOW)
                # 필요에 따라 추가 작업을 여기서 수행할 수 있습니다.

def main(args=None):
    rclpy.init(args=args)

    person_detection_node = PersonDetectionNode()

    rclpy.spin(person_detection_node)

    # 노드가 종료될 때 호출됩니다.
    person_detection_node.destroy_node()
    rclpy.shutdown()
    GPIO.cleanup()


if __name__ == '__main__':
    main()
