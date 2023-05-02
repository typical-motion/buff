#include <buff.h>
#include <message_filters/subscriber.h>
#include <uart_process_2/uart_send.h>

using namespace sensor_msgs;
using namespace message_filters;
float toFixed(double in) {
    float result = (int)(in * 100);
    return result / 100;
}
void Buff::targetTo(const EulerAngle& currentAngle, double distance, bool hasTarget, int attackFlag, int predictLatency) {
    uart_process_2::uart_send send_msg;
    send_msg.curYaw = toFixed(-currentAngle.yaw);
    send_msg.curPitch = toFixed(currentAngle.pitch);
    send_msg.curDistance = distance;
    send_msg.time = hasTarget;
    send_msg.attackFlag = 0x01;
    send_msg.predictLatency = predictLatency;
    this->uartPublisher.publish(send_msg);
    this->uartSent = send_msg;
}                    