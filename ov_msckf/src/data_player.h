#ifndef __DATA_PLAYER_H__
#define __DATA_PLAYER_H__
#include <fstream>
#include <opencv2/opencv.hpp>
#include "vs_os.h"

class VioDataPlayer
{
public:
    VioDataPlayer():exist_nav(false),m_is_open(false){}

    bool open(const char* datadir)
    {
        char video_file[128] = {0};
        char img_file[128] = {0};
        char imu_file[128] = {0};
        char nav_file[128] = {0};
        sprintf(video_file, "%s/img.avi", datadir);
        sprintf(img_file, "%s/imgts.txt", datadir);
        sprintf(imu_file, "%s/imu_meta.txt", datadir);
        sprintf(nav_file, "%s/nav.txt", datadir);
        if(vs::exists(nav_file) && vs::filesize(nav_file) > 1000)
        {
            exist_nav = true;
            return open(video_file, img_file, imu_file, nav_file);
        }
        return open(video_file, img_file, imu_file);
    }

    bool open(const char* video_file, const char* img_file, const char* imu_file)
    {
        m_is_open = false;
        m_cap.open(video_file);
        if(!m_cap.isOpened())
        {
            printf("[ERROR] stereo_video_play: cannot open video %s\n", video_file);
            return false;
        }
        m_fin_img.open(img_file);
        if(!m_fin_img.is_open())
        {
            printf("[ERROR] stereo_video_play: cannot open file %s\n", img_file);
            return false;
        }
        m_fin_imu.open(imu_file);
        if(!m_fin_imu.is_open())
        {
            printf("[ERROR] stereo_video_play: cannot open file %s\n", imu_file);
            return false;
        }
        m_is_open = true;
        return true;
    }

    bool open(const char* video_file, const char* img_file, const char* imu_file, const char* nav_file)
    {
        m_is_open = false;
        m_cap.open(video_file);
        if(!m_cap.isOpened())
        {
            printf("[ERROR] stereo_video_play: cannot open video %s\n", video_file);
            return false;
        }
        m_fin_img.open(img_file);
        if(!m_fin_img.is_open())
        {
            printf("[ERROR] stereo_video_play: cannot open file %s\n", img_file);
            return false;
        }
        m_fin_imu.open(imu_file);
        if(!m_fin_imu.is_open())
        {
            printf("[ERROR] stereo_video_play: cannot open file %s\n", imu_file);
            return false;
        }
        m_fin_nav.open(nav_file);
        if(!m_fin_nav.is_open())
        {
            printf("[ERROR] stereo_video_play: cannot open file %s\n", nav_file);
            return false;
        }
        m_is_open = true;
        return true;
    }

    bool isOpen() {return m_is_open;}

    bool readImu()
    {
        if(m_fin_imu.eof()) {return false;}
        m_fin_imu >> imu_ts >> imu_acc[0] >> imu_acc[1] >> imu_acc[2]
                    >> imu_gyro[0] >> imu_gyro[1] >> imu_gyro[2];
        return true;
    }

    bool readImage(double td = 0)
    {
        if(!readImgts(td) || !m_cap.read(img)) {return false;}
        if(exist_nav)
        {
            if(!readNav())
                return false;
        }
        cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
        // cv::resize(img, img, cv::Size(), 0.5, 0.5);
        int rows = img.rows/2;
        if(rows == 0) {return false;}
        imgl = img.rowRange(0,rows);
        imgr = img.rowRange(rows,rows*2);
        return true;
    }

    bool imuFirst()
    {
        return imu_ts < img_ts;
    }

    double  imu_ts;
    double  imu_acc[3];
    double  imu_gyro[3];
    double  img_ts;
    cv::Mat img, imgl, imgr;
    double ts;
    double nav_ts;
    int nav_status;
    int nav_light_mode;
    float nav_pos[3];
    float nav_e[3];
    float nav_vel[3];
    float vel_gps[3];
    double lati;
    double longi;
    double alti;
    double alti_msl;
    double height;
    bool exist_nav;

private:
    bool                m_is_open;
    cv::VideoCapture    m_cap;
    std::ifstream       m_fin_img;
    std::ifstream       m_fin_imu;
    std::ifstream       m_fin_nav;
    std::ifstream       m_fin_cam0;
    std::ifstream       m_fin_cam1;

    bool readImgts(double td = 0)
    {        
        if(m_fin_img.eof()) {return false;}
        m_fin_img >> img_ts;
        // int idx;
        // double temp_ts, left_id, right_id, left_ts, right_ts;
        // m_fin_img >> idx >> temp_ts >> left_id >> right_id >> left_ts >> right_ts;
        // img_ts = left_ts;
        img_ts += td;
        return true;
    }

    bool readNav(double td = 0)
    {
        if(m_fin_nav.eof()) {return false;}
        // m_fin_nav >> data_idx >> nav.ts >> nav.status >> nav.flight_mode>>
        //         nav.pos[0] >> nav.pos[1] >> nav.pos[2] >>
        //         nav.e[0] >> nav.e[1] >> nav.e[2]>>
        //         nav.vel[0] >> nav.vel[1] >> nav.vel[2] >>
        //         nav.vel_gps[0] >> nav.vel_gps[1] >> nav.vel_gps[2] >>
        //         nav.lati >> nav.longi>> nav.alti >> nav.alti_msl >>
        //         nav.height;
        m_fin_nav >> ts >> nav_ts >> nav_status >> nav_light_mode>>
                nav_pos[0] >> nav_pos[1] >> nav_pos[2] >>
                nav_e[0] >> nav_e[1] >> nav_e[2]>>
                nav_vel[0] >> nav_vel[1] >> nav_vel[2] >>
                vel_gps[0] >> vel_gps[1] >> vel_gps[2] >>
                lati >> longi>> alti >> alti_msl >>
                height;
        return true;
    }

};

#endif//__DATA_PLAYER_H__