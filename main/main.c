#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_attr.h"

#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}
#define constrain(amt, low, high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

#define GPIO_PWM0A_OUT 18   //Set GPIO 15 as PWM0A
#define GPIO_PWM0B_OUT 19   //Set GPIO 16 as PWM0B

#define GPIO_PWM1A_OUT 2    //Set GPIO 15 as PWM0A
#define GPIO_PWM1B_OUT 4    //Set GPIO 16 as PWM0B

rcl_publisher_t vel_pub;
rcl_subscription_t twist_sub;

geometry_msgs__msg__Twist sub_msg;
geometry_msgs__msg__Twist pub_msg;

float PWM_MOTOR_MIN = 0.0;
float PWM_MOTOR_MAX = 100.0;

static void mcpwm_example_gpio_initialize(void)
{
    printf("initializing mcpwm gpio...\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM0A_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, GPIO_PWM0B_OUT);    
	mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1A, GPIO_PWM1A_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1B, GPIO_PWM1B_OUT);
}

static void motor_move(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num , int dir , float duty_cycle)
{
    mcpwm_generator_t off_generator;
    mcpwm_generator_t on_generator;

    if (dir == 0){
        off_generator = MCPWM_OPR_A;
        on_generator = MCPWM_OPR_B;
    }
    else{
        off_generator = MCPWM_OPR_B;
        on_generator = MCPWM_OPR_A;
    }
    mcpwm_set_signal_low(mcpwm_num, timer_num, off_generator);
    mcpwm_set_duty(mcpwm_num, timer_num, on_generator, duty_cycle);
    mcpwm_set_duty_type(mcpwm_num, timer_num, on_generator, MCPWM_DUTY_MODE_0);  //call this each time, if operator was previously in low/high state
}

static void motor_stop()
{

    mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
    mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B);    
    mcpwm_set_signal_low(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A);
    mcpwm_set_signal_low(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_B);
}

float fmap(float val, float in_min, float in_max, float out_min, float out_max) {
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float fabsz(float val){
	if (val < 0){
		val = val * -1.0;
	}
	return val;
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	(void) last_call_time;
	if (timer != NULL) {
		pub_msg.linear.x = sub_msg.linear.x;
		pub_msg.angular.z = sub_msg.angular.z;
		RCSOFTCHECK(rcl_publish(&vel_pub, &pub_msg, NULL));
		//printf("Sent: %d\n",  (int)  pub_msg.data);
	}
}

void cmdvel_callback(const void * msgin)
{
	const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
	//printf("Received: %d\n",  (int)  msg->data);
	
	float linear = msg->linear.x;
    float angular = msg->angular.z;

	float left = (linear - angular) / 2.0f;
    float right = (linear + angular) / 2.0f;

	// 0 --> ccw, 1 --> cw

    // Then map those values to PWM intensities. PWM_MOTOR_MAX = full speed, PWM_MOTOR_MIN = the minimal amount of power at which the motors begin moving.
    float pwmLeft = fmap(fabsz(left), 0, 1, PWM_MOTOR_MIN, PWM_MOTOR_MAX);
    float pwmRight = fmap(fabsz(right), 0, 1, PWM_MOTOR_MIN, PWM_MOTOR_MAX);

	if (linear == 0 && angular == 0){
		motor_stop();
	}
	else{
		motor_move(MCPWM_UNIT_0, MCPWM_TIMER_0, left > 0, pwmLeft);
		motor_move(MCPWM_UNIT_1, MCPWM_TIMER_1, right > 0, pwmRight);
	}
}

void micro_ros_task(void * arg){
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// Create init_options.
	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	RCCHECK(rcl_init_options_init(&init_options, allocator));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
	rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

	// Static Agent IP and port can be used instead of autodisvery.
	RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
	//RCCHECK(rmw_uros_discover_agent(rmw_options));
#endif
	// Setup support structure.
	RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

	// Create node.
	rcl_node_t twist_node = rcl_get_zero_initialized_node();
	RCCHECK(rclc_node_init_default(&twist_node, "twist_sub_pub_node", "", &support));	

	// Create subscriber.
	RCCHECK(rclc_subscription_init_default(
		&twist_sub,
		&twist_node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
		"cmd_vel"));

	// Create publisher
	RCCHECK(rclc_publisher_init_default(
		&vel_pub,
		&twist_node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
		"vel_msgs"));

	// Create timer.
	rcl_timer_t timer = rcl_get_zero_initialized_timer();
	const unsigned int timer_timeout = 1000;
	RCCHECK(rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		timer_callback));

	rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
	RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
	unsigned int rcl_wait_timeout = 1000;   // in ms
	RCCHECK(rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(rcl_wait_timeout)));

	RCCHECK(rclc_executor_add_subscription(&executor, &twist_sub, &sub_msg, &cmdvel_callback, ON_NEW_DATA));
	RCCHECK(rclc_executor_add_timer(&executor, &timer));

	while(1){
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
		usleep(100);
	}
	
	RCCHECK(rcl_subscription_fini(&twist_sub, &twist_node));
	RCCHECK(rcl_publisher_fini(&vel_pub, &twist_node));
	RCCHECK(rcl_node_fini(&twist_node));

  	vTaskDelete(NULL);

}


void app_main(void)
{
	mcpwm_example_gpio_initialize();
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 1000;    //frequency = 500Hz,
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_1, &pwm_config);

#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif

    //pin micro-ros task in APP_CPU to make PRO_CPU to deal with wifi:
    xTaskCreate(micro_ros_task,
            "uros_task",
            CONFIG_MICRO_ROS_APP_STACK,
            NULL,
            CONFIG_MICRO_ROS_APP_TASK_PRIO,
            NULL);
}