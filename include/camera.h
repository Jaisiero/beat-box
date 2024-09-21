#pragma once

#include "defines.hpp"
#include "math.hpp"

const glm::vec3 INIT_FORWARD = {0, 0, 1};
const glm::vec3 INIT_CAMERA_POS = {0, 1, -4.5};
const glm::vec3 INIT_CAMERA_UP = {0, 1, 0};

const float INIT_CAMERA_FOV = 45.0f;
const float INIT_CAMERA_WIDTH = 800.0f;
const float INIT_CAMERA_HEIGHT = 600.0f;
const float INIT_CAMERA_NEAR = 0.001f;
const float INIT_CAMERA_FAR = 1000.0f;
const float CAMERA_SPEED = 0.01f;
const float MOUSE_SENSITIVITY = 0.005f;
const float SPEED_UP_MULTIPLIER = 10.0f;
const float CAMERA_DEF_FOCUS_DIST_MIN = 0.0f;
const float CAMERA_DEF_FOCUS_DIST_MAX = 100.0f;
const float CAMERA_DEF_FOCUS_ANGLE_MIN = 0.0f;
const float CAMERA_DEF_FOCUS_ANGLE_MAX = 5.0f;

const unsigned int TRIPPLE_BUFFER = 3;


typedef struct Camera {
    glm::vec3 forward;
    glm::vec3 position;
    glm::vec3 center;
    glm::vec3 up;
    float fov;
    unsigned int width;
    unsigned int height;
    float _near;
    float _far;
    glm::vec2 last_mouse_pos;
    float speed;
    bool mouse_left_press;
    bool mouse_middle_pressed;
    std::array<bool, TRIPPLE_BUFFER> moved;
    unsigned int frame_index;
    bool shift_status;
    float defocus_angle;
    float focus_dist;
} Camera;


void reset_camera(Camera& cam) {
    cam.forward = INIT_FORWARD;
    cam.position = INIT_CAMERA_POS;
    cam.up = INIT_CAMERA_UP;
    cam.fov = INIT_CAMERA_FOV;
    cam.width = INIT_CAMERA_WIDTH;
    cam.height = INIT_CAMERA_HEIGHT;
    cam._near = INIT_CAMERA_NEAR;
    cam._far = INIT_CAMERA_FAR;
    cam.speed = CAMERA_SPEED;
    cam.last_mouse_pos = glm::vec2(0, 0);
    cam.mouse_left_press = false;
    cam.mouse_middle_pressed = false;
    cam.moved[0] = cam.moved[1] = cam.moved[2] = true;
    cam.frame_index = 0;
    cam.shift_status = false;
    cam.defocus_angle = CAMERA_DEF_FOCUS_ANGLE_MIN;
    cam.focus_dist = CAMERA_DEF_FOCUS_DIST_MIN;
}

const glm::mat4 _get_view_matrix(const Camera& cam) {
    return glm::lookAt(cam.position, cam.position + cam.forward, cam.up);
}

const glm::mat4 _get_projection_matrix(const Camera& cam, bool flip_Y = false) {
    auto proj_mat = glm::perspectiveRH_ZO(cam.fov, cam.width / (float)cam.height , cam._near, cam._far);
    if(flip_Y) 
        proj_mat[1][1] *= -1;
    return proj_mat;
}

const glm::mat4 _get_view_projection_matrix(const Camera& cam, bool flip_Y = false) {
    return _get_projection_matrix(cam, flip_Y) * _get_view_matrix(cam);
}

const glm::mat4 _get_inverse_view_matrix(const Camera& cam) {
    return glm::inverse(_get_view_matrix(cam));
}

const glm::mat4 _get_inverse_projection_matrix(const Camera& cam, bool flip_Y = false) {
    return glm::inverse(_get_projection_matrix(cam, flip_Y));
}

const glm::mat4 _get_inverse_view_projection_matrix(const Camera& cam, bool flip_Y = false) {
    return glm::inverse(_get_view_projection_matrix(cam, flip_Y));
}

const daxa_f32mat4x4 get_view_matrix(const Camera& cam) {
    return daxa_mat4_from_glm_mat4(_get_view_matrix(cam));
}

const daxa_f32mat4x4 get_projection_matrix(const Camera& cam, bool flip_Y = false) {
    return daxa_mat4_from_glm_mat4(_get_projection_matrix(cam, flip_Y));
}

const daxa_f32mat4x4 get_view_projection_matrix(const Camera& cam, bool flip_Y = false) {
    return daxa_mat4_from_glm_mat4(_get_view_projection_matrix(cam, flip_Y));
}

const daxa_f32mat4x4 get_inverse_view_matrix(const Camera& cam) {
    return daxa_mat4_from_glm_mat4(_get_inverse_view_matrix(cam));
}

const daxa_f32mat4x4 get_inverse_projection_matrix(const Camera& cam, bool flip_Y = false) {
    return daxa_mat4_from_glm_mat4(_get_inverse_projection_matrix(cam, flip_Y));
}

const daxa_f32mat4x4 get_inverse_view_projection_matrix(const Camera& cam, bool flip_Y = false) {
    return daxa_mat4_from_glm_mat4(_get_inverse_view_projection_matrix(cam, flip_Y));
}

const glm::vec3 camera_get_direction(const Camera& cam) {
    return cam.forward;
}

const glm::vec3 camera_get_right(const Camera& cam) {
    return glm::normalize(glm::cross(camera_get_direction(cam), cam.up));
}

const glm::vec3 camera_get_up(const Camera& cam) {
    return glm::normalize(glm::cross(camera_get_right(cam), camera_get_direction(cam)));
}

const glm::vec3& camera_get_position(const Camera& cam) {
    return cam.position;
}

const glm::vec3& camera_get_center(const Camera& cam) {
    return cam.center;
}

const glm::vec3& camera_get_up_vector(const Camera& cam) {
    return cam.up;
}

const float& camera_get_fov(const Camera& cam) {
    return cam.fov;
}

const float camera_get_aspect(const Camera& cam) {
    return cam.width / cam.height;
}

const unsigned int& camera_get_width(const Camera& cam) {
    return cam.width;
}

const unsigned int& camera_get_height(const Camera& cam) {
    return cam.height;
}

const float& camera_get__near(const Camera& cam) {
    return cam._near;
}

const float& camera_get_far(const Camera& cam) {
    return cam._far;
}

const float& camera_get_speed(const Camera& cam) {
    return cam.speed;
}

const bool& camera_get_moved(const Camera& cam) {
    return cam.moved[cam.frame_index];
}

const glm::vec2& camera_get_last_mouse_pos(const Camera& cam) {
    return cam.last_mouse_pos;
}

const bool& camera_get_mouse_left_press(const Camera& cam) {
    return cam.mouse_left_press;
}

const bool& camera_get_mouse_middle_pressed(const Camera& cam) {
    return cam.mouse_middle_pressed;
}

const bool& camera_get_shift_status(const Camera& cam) {
    return cam.shift_status;
}

const float& camera_get_defocus_angle(const Camera& cam) {
    return cam.defocus_angle;
}

const float& camera_get_focus_dist(const Camera& cam) {
    return cam.focus_dist;
}





void move_camera(Camera& cam, const glm::vec3& direction) {
    cam.position += direction * cam.speed * (cam.shift_status ? SPEED_UP_MULTIPLIER : 1.0f);
    cam.moved[0] = cam.moved[1] = cam.moved[2] = true;
}

void move_camera_forward(Camera& cam) {
    move_camera(cam,  camera_get_direction(cam));
}

void move_camera_backward(Camera& cam) {
    move_camera(cam, -camera_get_direction(cam));
}

void move_camera_right(Camera& cam) {
    move_camera(cam,  camera_get_right(cam));
}

void move_camera_left(Camera& cam) {
    move_camera(cam, -camera_get_right(cam));
}

void move_camera_up(Camera& cam) {
    move_camera(cam, camera_get_up(cam));
}

void move_camera_down(Camera& cam) {
    move_camera(cam, -camera_get_up(cam));
}

void camera_set_position(Camera& cam, const glm::vec3& position) {
    cam.position = position;
}

void camera_set_center(Camera& cam, const glm::vec3& center) {
    cam.center = center;
}

void camera_set_up_vector(Camera& cam, const glm::vec3& up) {
    cam.up = up;
}

void camera_set_fov(Camera& cam, float fov) {
    cam.fov = fov;
}

void camera_set_aspect(Camera& cam, unsigned int width, unsigned int height) {
    cam.width = width;
    cam.height = height;
}

void camera_set_near(Camera& cam, float near_plane) {
    cam._near = near_plane;
}

void camera_set_far(Camera& cam, float far_plane) {
    cam._far = far_plane;
}

void camera_shift_pressed(Camera& cam) {
    cam.shift_status = true;
}

void camera_shift_released(Camera& cam) {
    cam.shift_status = false;
}

void camera_set_speed(Camera& cam, float speed) {
    cam.speed = speed;
}

void camera_set_moved(Camera& cam) {
    cam.moved[0] = cam.moved[1] = cam.moved[2] = true;
}

void camera_reset_moved(Camera& cam) {
    cam.moved[cam.frame_index] = false;
    cam.frame_index = (cam.frame_index + 1) % TRIPPLE_BUFFER;
}

void camera_set_last_mouse_pos(Camera& cam, const glm::vec2& last_mouse_pos) {
    cam.last_mouse_pos = last_mouse_pos;
}

void camera_set_mouse_left_press(Camera& cam, bool mouse_left_press) {
    cam.mouse_left_press = mouse_left_press;
}

void camera_set_mouse_middle_pressed(Camera& cam, bool mouse_middle_pressed) {
    cam.mouse_middle_pressed = mouse_middle_pressed;
}

void camera_set_defocus_angle(Camera& cam, float defocus_angle) {
    cam.defocus_angle = std::max(CAMERA_DEF_FOCUS_ANGLE_MIN, std::min(CAMERA_DEF_FOCUS_ANGLE_MAX, defocus_angle));
}

void camera_set_focus_dist(Camera& cam, float focus_dist) {
    cam.focus_dist = std::max(CAMERA_DEF_FOCUS_DIST_MIN, std::min(CAMERA_DEF_FOCUS_DIST_MAX, focus_dist));
}

// rotate camera around its center
void rotate_camera(Camera& cam, float currentX, float currentY)
{
    glm::vec3 up_direction = glm::vec3(0.0f, 1.0f, 0.0f);
    glm::vec3 right_direction = glm::cross(cam.forward, up_direction);

    // check if last mouse position is set
    if (cam.last_mouse_pos.x == 0.0f && cam.last_mouse_pos.y == 0.0f)
    {
        camera_set_last_mouse_pos(cam, glm::vec2(currentX, currentY));
        return;
    }

    // Calculate the change in mouse position
    float deltaX = (currentX - cam.last_mouse_pos.x);
    float deltaY = (currentY - cam.last_mouse_pos.y);


    camera_set_last_mouse_pos(cam, glm::vec2(currentX, currentY));


    // Rotation
	if (deltaX != 0.0f || deltaY != 0.0f)
	{
		float pitch_delta = deltaY * MOUSE_SENSITIVITY;
		float yaw_delta = deltaX * MOUSE_SENSITIVITY;

		glm::quat q = glm::normalize(glm::cross(glm::angleAxis(-pitch_delta, right_direction),
			glm::angleAxis(-yaw_delta, glm::vec3(0.f, 1.0f, 0.0f))));
		cam.forward = glm::rotate(q, cam.forward);

        camera_set_moved(cam);
	}
}

void rotate_camera_yaw(Camera& cam, float yaw) {
    rotate_camera(cam, yaw, 0.0f);
}

void rotate_camera_pitch(Camera& cam, float pitch) {
    rotate_camera(cam, 0.0f, pitch);
}

void camera_set_mouse_delta(Camera& cam, const glm::vec2& mouse_delta) {
    if(cam.mouse_left_press) {
        rotate_camera(cam, mouse_delta.x, mouse_delta.y);
    }
}