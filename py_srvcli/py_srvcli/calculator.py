from pack.srv import FoundPerson
import rclpy
from rclpy.node import Node


class Calculator(Node):

    def __init__(self):
        super().__init__('calculator')  # 노드 이름

        # 서비스 서버 생성
        self.service = self.create_service(
            FoundPerson, 'found_person_service', self.handle_found_person)

    def handle_found_person(self, request, response):
        # 요청 처리 로직
        self.get_logger().info(f'Received request: {request.request}')

        # 사용자 입력 대기
        input("Press Enter to send a response...")  # 엔터 키 입력 대기

        # 응답 메시지
        response.response = "Found Missing Person!"
        # self.get_logger().info("Response sent.")
        return response


def main(args=None):
    rclpy.init(args=args)
    calculator = Calculator()

    try:
        rclpy.spin(calculator)  # 서비스 서버 실행
    except KeyboardInterrupt:
        calculator.get_logger().info('Keyboard Interrupt')
    finally:
        calculator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
