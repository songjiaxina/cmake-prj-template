#ifndef PRJ_SOURCE_H_
#define PRJ_SOURCE_H_

#include <atomic>
#include <chrono>
#include <iomanip>
#include <iostream>

// Disabled since it was causing problems within Docker.
//#include <backward.hpp>
#include <gflags/gflags.h>
#include <opencv2/opencv.hpp>
#include <opencv/highgui.h>
#include <pangolin/pangolin.h>

namespace gui {

using namespace std;
/// \brief The main GUI and entry point for DynSLAM.
class PangolinGui {
 public:
    PangolinGui() : width_(1283), height_(356) { CreatePangolinDisplays(); }

    PangolinGui(const PangolinGui &) = delete;
    PangolinGui(PangolinGui &&) = delete;
    PangolinGui &operator=(const PangolinGui &) = delete;
    PangolinGui &operator=(PangolinGui &&) = delete;

    virtual ~PangolinGui() {}

    void DrawPoses(long current_time_ms) {}

    /// \brief Executes the main Pangolin input and rendering loop.
    void Run() {
        // Default hooks for exiting (Esc) and fullscreen (tab).
        while (!pangolin::ShouldQuit()) {
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            glColor3f(1.0, 1.0, 1.0);
            pangolin::GlFont &font = pangolin::GlFont::I();

            main_view_->Activate(*pane_cam_);
            glEnable(GL_DEPTH_TEST);
            glColor3f(1.0f, 1.0f, 1.0f);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            glDisable(GL_DEPTH_TEST);
            glDepthMask(false);

            if (1) {
                //   font.Text(message).Draw(-0.90f, 0.80f);
            }
            //*/
            font.Text("Frame #%d", 89).Draw(-0.90f, 0.90f);

            rgb_view_.Activate();
            glColor3f(1.0f, 1.0f, 1.0f);

            detail_views_->Activate();

            depth_view_.Activate();
            segment_view_.Activate();
            object_view_.Activate();
            object_reconstruction_view_.Activate();

            pangolin::FinishFrame();
        }
    }

    /// \brief Renders informative labels for the currently active track.
    /// Meant to be rendered over the segmentation preview window pane.
    void DrawInstanceLables() {
        //  pangolin::GlFont &font = pangolin::GlFont::I();

        //  auto &instanceTracker =
        //  dyn_slam_->GetInstanceReconstructor()->GetInstanceTracker(); for
        //  (const auto &pair: instanceTracker.GetActiveTracks()) {
        //    Track &track = instanceTracker.GetTrack(pair.first);
        //    // Nothing to do for tracks with we didn't see this frame.
        //    if (track.GetLastFrame().frame_idx !=
        //    dyn_slam_->GetCurrentFrameNo() - 2) { // TODO(andrei): Why this
        //    index gap of 2?
        //      continue;
        //    }

        //    InstanceDetection latest_detection =
        //    track.GetLastFrame().instance_view.GetInstanceDetection(); const
        //    auto &bbox = latest_detection.copy_mask->GetBoundingBox(); auto
        //    gl_pos = utils::PixelsToGl(Eigen::Vector2f(bbox.r.x0, bbox.r.y0 -
        //    font.Height()),
        //                                    Eigen::Vector2f(width_, height_),
        //                                    Eigen::Vector2f(segment_view_.GetBounds().w,
        //                                                    segment_view_.GetBounds().h));

        //    stringstream info_label;
        //    info_label << latest_detection.GetClassName() << "#" <<
        //    track.GetId()
        //               << " [" << track.GetStateLabel().substr(0, 1) << "].";
        //    glColor3f(1.0f, 0.0f, 0.0f);
        //    font.Text(info_label.str()).Draw(gl_pos[0], gl_pos[1], 0);
        //  }
    }

 protected:
    /// \brief Creates the GUI layout and widgets.
    /// \note The layout is biased towards very wide images (~2:1 aspect ratio
    /// or more), which is very common in autonomous driving datasets.
    void CreatePangolinDisplays() {
        pangolin::CreateWindowAndBind(
            "DynSLAM GUI", 950,
            // One full-height pane with the main preview, plus 3 * 0.5
            // height ones for various visualizations.
            static_cast<int>(ceil(height_ * 2.5)));

        // 3D Mouse handler requires depth testing to be enabled
        glEnable(GL_DEPTH_TEST);

        // Issue specific OpenGl we might need
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        /***************************************************************************
         * GUI Buttons
         **************************************************************************/
        pangolin::CreatePanel("ui").SetBounds(0.0, 1.0, 0.0,
                                              pangolin::Attach::Pix(150));

        auto next_frame = [this]() {
            *(this->autoplay_) = false;
            this->ProcessFrame();
        };
        pangolin::Var<function<void(void)>> next_frame_button("ui.[N]ext Frame",
                                                              next_frame);
        pangolin::RegisterKeyPressCallback('n', next_frame);

        auto quit = [this]() { pangolin::QuitAll(); };
        pangolin::Var<function<void(void)>> quit_button("ui.[Q]uit", quit);
        pangolin::RegisterKeyPressCallback('q', quit);

        auto previous_preview_type = [this]() {

        };
        auto next_preview_type = [this]() {

        };
        pangolin::Var<function<void(void)>> ppt("ui.Previous Preview Type [j]",
                                                previous_preview_type);
        pangolin::RegisterKeyPressCallback('j', previous_preview_type);
        pangolin::Var<function<void(void)>> npt("ui.Next Preview Type [k]",
                                                next_preview_type);
        pangolin::RegisterKeyPressCallback('k', next_preview_type);

        display_raw_previews_ =
            new pangolin::Var<bool>("ui.Raw Previews", false, true);
        preview_sf_ =
            new pangolin::Var<bool>("ui.Show Scene Flow", false, true);

        pangolin::RegisterKeyPressCallback('r', [&]() {
            *display_raw_previews_ = !display_raw_previews_->Get();
        });

        // This constructs an OpenGL projection matrix from a calibrated camera
        // pinhole projection matrix. They are quite different, and the
        // conversion between them is nontrivial. See
        // https://ksimek.github.io/2013/06/03/calibrated_cameras_in_opengl/ for
        // more info.
        //  const Eigen::Matrix34f real_cam_proj =
        //  dyn_slam_->GetLeftRgbProjectionMatrix();
        float near = 0.01;
        float far = 1000.0f;
        // -y is up
        proj_ = pangolin::ProjectionMatrixRDF_TopLeft(width_, height_, 1, 2, 3,
                                                      4, near, far);

        pane_cam_ = new pangolin::OpenGlRenderState(
            proj_,
            pangolin::ModelViewLookAtRDF(0, -1.5, 15, 0, -1.5, 50, 0, 1, 0));
        instance_cam_ = new pangolin::OpenGlRenderState(
            proj_, pangolin::ModelViewLookAtRDF(-0.8, -0.20, -3, -0.8, -0.20,
                                                15, 0, 1, 0));

        float aspect_ratio = static_cast<float>(width_) / height_;
        rgb_view_ = pangolin::Display("rgb").SetAspect(aspect_ratio);
        depth_view_ = pangolin::Display("depth").SetAspect(aspect_ratio);

        segment_view_ = pangolin::Display("segment").SetAspect(aspect_ratio);
        object_view_ = pangolin::Display("object").SetAspect(aspect_ratio);
        float camera_translation_scale = 1.0f;
        float camera_zoom_scale = 1.0f;

        // These objects remain under Pangolin's management, so they don't need
        // to be deleted by the current class.
        main_view_ = &(pangolin::Display("main").SetAspect(aspect_ratio));
        //  main_view_->SetHandler(
        //      new DSHandler3D(pane_cam_,
        //                      pangolin::AxisY,
        //                      camera_translation_scale,
        //                      camera_zoom_scale));

        detail_views_ = &(pangolin::Display("detail"));

        // Add labels to our data logs (and automatically to our plots).
        data_log_.SetLabels({
            "Active tracks",
            "Free GPU Memory (100s of MiB)",
            "Static map memory usage (100s of MiB)",
            "Static map memory usage without decay (100s of Mib)",
        });

        // OpenGL 'view' of data such as the number of actively tracked
        // instances over time.
        float tick_x = 1.0f;
        float tick_y = 1.0f;
        plotter_ = new pangolin::Plotter(&data_log_, 0.0f, 200.0f, -0.1f, 25.0f,
                                         tick_x, tick_y);
        plotter_->Track(
            "$i");  // This enables automatic scrolling for the live plots.

        // TODO(andrei): Maybe wrap these guys in another controller, make it an
        // equal layout and automagically support way more aspect ratios?
        main_view_->SetBounds(pangolin::Attach::Pix(height_ * 1.5), 1.0,
                              pangolin::Attach::Pix(150), 1.0);
        detail_views_->SetBounds(0.0, pangolin::Attach::Pix(height_ * 1.5),
                                 pangolin::Attach::Pix(150), 1.0);
        detail_views_->SetLayout(pangolin::LayoutEqual)
            .AddDisplay(rgb_view_)
            .AddDisplay(depth_view_)
            .AddDisplay(segment_view_)
            .AddDisplay(object_view_)
            .AddDisplay(*plotter_)
            .AddDisplay(object_reconstruction_view_);

        // Internally, InfiniTAM stores these as RGBA, but we discard the alpha
        // when we upload the textures for visualization (hence the 'GL_RGB'
        // specification).
        this->pane_texture_ = new pangolin::GlTexture(
            width_, height_, GL_RGB, false, 0, GL_RGB, GL_UNSIGNED_BYTE);
        this->pane_texture_mono_uchar_ = new pangolin::GlTexture(
            width_, height_, GL_RGB, false, 0, GL_RED, GL_UNSIGNED_BYTE);

        cout << "Pangolin UI setup complete." << endl;
    }

    //   void SelectNextVisualizedObject() {
    //     // We pick the closest next object (by ID). We need to do this
    //     because some tracks may no
    //     // longer be available.
    //    //  InstanceTracker &tracker =
    //    dyn_slam_->GetInstanceReconstructor()->GetInstanceTracker();
    //     int closest_next_id = -1;
    //     int closest_next_delta = std::numeric_limits<int>::max();

    //     visualized_object_idx_ = closest_next_id;
    //   }

    /// \brief Advances to the next input frame, and integrates it into the map.
    void ProcessFrame() {
        //  cout << endl << "[Starting frame " << dyn_slam_->GetCurrentFrameNo()
        //  + 1 << "]" << endl; active_object_count_ =
        //  dyn_slam_->GetInstanceReconstructor()->GetActiveTrackCount();

        //  if (! dyn_slam_input_->HasMoreImages() && FLAGS_close_on_complete) {
        //    cerr << "No more images, and I'm instructed to shut down when that
        //    happens. Bye!" << endl; pangolin::QuitAll(); return;
        //  }

        //  size_t free_gpu_memory_bytes;
        //  size_t total_gpu_memory_bytes;
        //  cudaMemGetInfo(&free_gpu_memory_bytes, &total_gpu_memory_bytes);

        //  const double kBytesToGb = 1.0 / 1024.0 / 1024.0 / 1024.0;
        //  double free_gpu_gb = static_cast<float>(free_gpu_memory_bytes) *
        //  kBytesToGb; data_log_.Log(
        //      active_object_count_,
        //      static_cast<float>(free_gpu_gb) * 10.0f,   // Mini-hack to make
        //      the scales better dyn_slam_->GetStaticMapMemoryBytes() * 10.0f *
        //      kBytesToGb, (dyn_slam_->GetStaticMapMemoryBytes() +
        //      dyn_slam_->GetStaticMapSavedDecayMemoryBytes()) * 10.0f *
        //      kBytesToGb
        //  );

        //  Tic("DynSLAM frame");
        //  // Main workhorse function of the underlying SLAM system.
        //  dyn_slam_->ProcessFrame(this->dyn_slam_input_);
        //  int64_t frame_time_ms = Toc(true);
        //  float fps = 1000.0f / static_cast<float>(frame_time_ms);
        //  cout << "[Finished frame " << dyn_slam_->GetCurrentFrameNo() << " in
        //  " << frame_time_ms
        //       << "ms @ " << setprecision(4) << fps << " FPS (approx.)]"
        //       << endl;
    }

    static void DrawOutlinedText(
        cv::Mat &target, const string &text, int x, int y, float scale = 1.5f) {
        int thickness = static_cast<int>(round(1.1 * scale));
        int outline_factor = 3;
        cv::putText(target, text, cv::Point_<int>(x, y),
                    cv::FONT_HERSHEY_DUPLEX, scale, cv::Scalar(0, 0, 0),
                    outline_factor * thickness, CV_AA);
        cv::putText(target, text, cv::Point_<int>(x, y),
                    cv::FONT_HERSHEY_DUPLEX, scale, cv::Scalar(230, 230, 230),
                    thickness, CV_AA);
    }

 private:
    /// Input frame dimensions. They dictate the overall window size.
    int width_, height_;

    pangolin::View *main_view_;
    pangolin::View *detail_views_;
    pangolin::View rgb_view_;
    pangolin::View depth_view_;
    pangolin::View segment_view_;
    pangolin::View object_view_;
    pangolin::View object_reconstruction_view_;

    pangolin::OpenGlMatrix proj_;
    pangolin::OpenGlRenderState *pane_cam_;
    pangolin::OpenGlRenderState *instance_cam_;

    // Graph plotter and its data logger object
    pangolin::Plotter *plotter_;
    pangolin::DataLog data_log_;

    pangolin::GlTexture *pane_texture_;
    pangolin::GlTexture *pane_texture_mono_uchar_;

    pangolin::Var<string> *reconstructions;

    // Atomic because it gets set from a UI callback. Technically, Pangolin
    // shouldn't invoke callbacks from a different thread, but using atomics for
    // this is generally a good practice anyway.
    atomic<int> active_object_count_;

    /// \brief When this is on, the input gets processed as fast as possible,
    /// without requiring any user input.
    pangolin::Var<bool> *autoplay_;
    /// \brief Whether to display the RGB and depth previews directly from the
    /// input, or from the static scene, i.e., with the dynamic objects removed.
    pangolin::Var<bool> *display_raw_previews_;
    /// \brief Whether to preview the sparse scene flow on the input and current
    /// instance RGP panes.
    pangolin::Var<bool> *preview_sf_;

    // TODO(andrei): Reset button.

    // Indicates which object is currently being visualized in the GUI.
    int visualized_object_idx_ = 0;

    /// \brief Prepares the contents of an OpenCV Mat object for rendering with
    /// Pangolin (OpenGL). Does not actually render the texture.
    static void UploadCvTexture(const cv::Mat &mat,
                                pangolin::GlTexture &texture,
                                bool color,
                                GLenum data_type) {
        int old_alignment, old_row_length;
        glGetIntegerv(GL_UNPACK_ALIGNMENT, &old_alignment);
        glGetIntegerv(GL_UNPACK_ROW_LENGTH, &old_row_length);

        int new_alignment = (mat.step & 3) ? 1 : 4;
        int new_row_length = static_cast<int>(mat.step / mat.elemSize());
        glPixelStorei(GL_UNPACK_ALIGNMENT, new_alignment);
        glPixelStorei(GL_UNPACK_ROW_LENGTH, new_row_length);

        // [RIP] If left unspecified, Pangolin assumes your texture type is
        // single-channel luminance, so you get dark, uncolored images.
        GLenum data_format = (color) ? GL_BGR : GL_LUMINANCE;
        texture.Upload(mat.data, data_format, data_type);

        glPixelStorei(GL_UNPACK_ALIGNMENT, old_alignment);
        glPixelStorei(GL_UNPACK_ROW_LENGTH, old_row_length);
    }
};

}  // namespace gui
#endif  // PRJ_SOURCE_H_