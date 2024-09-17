#pragma once

#include "defines.hpp"

BB_NAMESPACE_BEGIN

template <typename TaskType, typename Callback>
struct TaskTemplate : public TaskType
{
    using AttachmentViews = typename TaskType::AttachmentViews;

    // Stores the attachment views
    AttachmentViews views = {};

    // The user-defined callback function
    Callback user_callback;

    // Constructor to set up views and user-defined callback
    TaskTemplate(
        AttachmentViews const& views_,
        Callback user_callback_)
        : views(views_), user_callback(user_callback_) {}

    // The callback function called by the task graph
    void callback(daxa::TaskInterface ti)
    {
        // Call the user-defined callback function, passing in 'this' as 'self'
        user_callback(ti, *this);
    }
};

BB_NAMESPACE_END