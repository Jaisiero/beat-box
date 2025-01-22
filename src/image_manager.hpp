#pragma once
#include "defines.hpp"
#include "image.hpp"
#include "task_manager.hpp"

BB_NAMESPACE_BEGIN

// TODO: parameterize this
constexpr daxa::u32 SPATIOTEMPORAL_BLUE_NOISE_COUNT = 64;
constexpr daxa::usize SPATIOTEMPORAL_BLUE_NOISE_SIZE = 128 * 128 * 4;
constexpr daxa::usize INIT_HOST_BUFFER_SIZE = SPATIOTEMPORAL_BLUE_NOISE_SIZE * SPATIOTEMPORAL_BLUE_NOISE_COUNT;
constexpr char const *SPATIOTEMPORAL_BLUE_NOISE_COSINE = "stbn_vec2_2Dx1D_128x128x64_";

struct ImageManager
{
    // Constructor
    ImageManager(std::shared_ptr<GPUcontext> gpu,
                 std::shared_ptr<TaskManager> task_manager) : gpu(gpu), task_manager(task_manager) {}

    // Destructor
    ~ImageManager() {}

    // Create image manager
    bool create()
    {
        if (initialized)
        {
            return false;
        }

        // TODO: parameterize this
        // Create the texture
        stbn_texture = gpu->device.create_image({
            .flags = daxa::ImageCreateFlagBits::COMPATIBLE_2D_ARRAY,
            .dimensions = 3,
            .format = daxa::Format::R8G8B8A8_UNORM,
            .size = daxa::Extent3D(128, 128, SPATIOTEMPORAL_BLUE_NOISE_COUNT),
            .usage = daxa::ImageUsageFlagBits::TRANSFER_DST | daxa::ImageUsageFlagBits::SHADER_STORAGE | daxa::ImageUsageFlagBits::SHADER_SAMPLED,
            .name = "stbn_texture",
        });

        // Create images
        for (auto i = 0u; i < SPATIOTEMPORAL_BLUE_NOISE_COUNT; i++)
        {
            images.push_back(std::make_shared<BBImageTexture>(std::string(SPATIOTEMPORAL_BLUE_NOISE_COSINE) + std::to_string(i) + ".png", SPATIOTEMPORAL_BLUE_NOISE_PATH));
        }

        // Copy the image to host buffer
        image_host_buffer = gpu->device.create_buffer({
            .size = INIT_HOST_BUFFER_SIZE,
            .allocate_info = daxa::MemoryFlagBits::HOST_ACCESS_SEQUENTIAL_WRITE,
            .name = "image_host_buffer",
        });

        record_task_graph_upload_image(upload_images_TG);
        upload_images_TG.submit();
        upload_images_TG.complete();

        return initialized = true;
    }

    void record_task_graph_upload_image(TaskGraph &UI_TG)
    {
        daxa::InlineTaskInfo task_upload_image({
            .attachments = {
                daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_READ, task_host_buffer),
                daxa::inl_attachment(daxa::TaskImageAccess::TRANSFER_WRITE, task_image),
            },
            .task = [this](daxa::TaskInterface const &ti)
            {
                ti.recorder.pipeline_barrier({
                    .dst_access = daxa::AccessConsts::TRANSFER_WRITE,
                });

                ti.recorder.copy_buffer_to_image({
                    .buffer = ti.get(task_host_buffer).ids[0],
                    .buffer_offset = 0,
                    .image = ti.get(task_image).ids[0],
                    .image_offset = daxa::Offset3D(0, 0, 0),
                    .image_extent = daxa::Extent3D(128, 128, 64),
                });
                
            },
            .name = "upload image",
        });
        std::array<daxa::TaskBuffer, 1> buffers = {
            task_host_buffer};
        std::array<daxa::TaskImage, 1> images = {
            task_image};
        std::array<daxa::InlineTaskInfo, 1> tasks = {
            task_upload_image};
        UI_TG = task_manager->create_task_graph("Upload Image Task Graph", std::span<daxa::InlineTaskInfo>(tasks), std::span<daxa::TaskBuffer>(buffers), std::span<daxa::TaskImage>(images), {}, {});
    }

    void destroy()
    {
        if (!initialized)
        {
            return;
        }

        // Destroy the texture
        gpu->device.destroy_image(stbn_texture);

        // Destroy the host buffer
        gpu->device.destroy_buffer(image_host_buffer);

        // Destroy the images
        for (auto &image : images)
        {
            image.reset();
        }

        initialized = false;
    }

    // Upload an image
    // TODO: parameterize this
    bool upload_images()
    {
        if (!initialized)
        {
            return false;
        }

        usize offset = 0;
        u32 image_index = 0;
        for (auto &image : images)
        {
            upload_image(image, image_index++, offset);
        }

        task_host_buffer.set_buffers({.buffers = std::array{image_host_buffer}});

        task_image.set_images({.images = std::array{stbn_texture}});


        // // Copy the image data to the texture
        // auto rec = gpu->device.create_transfer_command_recorder({daxa::QueueFamily::TRANSFER});

        // rec.pipeline_barrier_image_transition({
        //     .dst_access = daxa::AccessConsts::TRANSFER_WRITE,
        //     .dst_layout = daxa::ImageLayout::TRANSFER_DST_OPTIMAL,
        //     .image_id = stbn_texture,
        // });

        // upload_image_gpu(rec);

        // rec.pipeline_barrier({daxa::AccessConsts::TRANSFER_WRITE, daxa::AccessConsts::TRANSFER_READ_WRITE});
        // auto commands = rec.complete_current_commands();
        // gpu->device.submit_commands({
        //     .queue = daxa::QUEUE_TRANSFER_0,
        //     .command_lists = std::array{commands},
        // });

        // // Wait for the transfer to finish
        // gpu->device.queue_wait_idle(daxa::QUEUE_TRANSFER_0);

        upload_images_TG.execute();

        if(offset != INIT_HOST_BUFFER_SIZE)
        {
            return false;
        }

        return true;
    }

    // Upload an image
    void upload_image(std::shared_ptr<BBImageTexture> image, u32 image_index, usize& offset)
    {

        auto image_size = image->get_size();
        // Create the texture
        auto stbn_host_address = gpu->device.buffer_host_address(image_host_buffer).value() + offset;
        std::memcpy(stbn_host_address, image->get_data(), image_size);

        offset += image_size;
    }

    // Upload an image
    void upload_image_gpu(daxa::TransferCommandRecorder& rec)
    {
        rec.pipeline_barrier({
            .src_access = daxa::AccessConsts::TRANSFER_READ_WRITE,
            .dst_access = daxa::AccessConsts::TRANSFER_WRITE,
        });

        rec.copy_buffer_to_image({
            .buffer = image_host_buffer,
            .buffer_offset = 0,
            .image = stbn_texture,
            .image_offset = daxa::Offset3D(0, 0, 0),
            .image_extent = daxa::Extent3D(128, 128, 64),
        });
    }

    // Get the image
    daxa::ImageId get_spatiotemporal_blue_noise_image() const
    {
        return stbn_texture;
    }

private:
    daxa::ImageId stbn_texture;
    // Host buffer for uploading images
    daxa::BufferId image_host_buffer;
    // Gpu context reference
    std::shared_ptr<GPUcontext> gpu;
    // Task manager reference
    std::shared_ptr<TaskManager> task_manager;
    // Initialization flag
    bool initialized = false;
    // Task graph for uploading images
    TaskGraph upload_images_TG;
    // Task buffer upload image
    daxa::TaskBuffer task_host_buffer{{.initial_buffers = {}, .name = "task_host_image_buffer"}};
    // Task image upload image
    daxa::TaskImage task_image{{.initial_images = {}, .name = "task_upload_image"}};

    // Image references
    std::vector<std::shared_ptr<BBImageTexture>> images;
};

BB_NAMESPACE_END
