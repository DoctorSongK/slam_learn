//
// Created by xiang on 2022/1/4.
//

#include "ch3/utm_convert.h"
#include "common/math_utils.h"
#include "utm_convert/utm.h"

#include <glog/logging.h>

namespace sad {

bool LatLon2UTM(const Vec2d& latlon, UTMCoordinate& utm_coor) {
    long zone = 0;
    char char_north = 0;
    long ret = Convert_Geodetic_To_UTM(latlon[0] * math::kDEG2RAD, latlon[1] * math::kDEG2RAD, &zone, &char_north,
                                       &utm_coor.xy_[0], &utm_coor.xy_[1]);
    utm_coor.zone_ = (int)zone;
    utm_coor.north_ = char_north == 'N';

    return ret == 0;
}

bool UTM2LatLon(const UTMCoordinate& utm_coor, Vec2d& latlon) {
    bool ret = Convert_UTM_To_Geodetic((long)utm_coor.zone_, utm_coor.north_ ? 'N' : 'S', utm_coor.xy_[0],
                                       utm_coor.xy_[1], &latlon[0], &latlon[1]);
    latlon *= math::kRAD2DEG;
    return ret == 0;
}

bool ConvertGps2UTM(GNSS& gps_msg, const Vec2d& antenna_pos, const double& antenna_angle, const Vec3d& map_origin) {
    /// 经纬高转换为UTM
    UTMCoordinate utm_rtk;
    if (!LatLon2UTM(gps_msg.lat_lon_alt_.head<2>(), utm_rtk)) {
        return false;
    }
    utm_rtk.z_ = gps_msg.lat_lon_alt_[2];

    /// GPS heading 转成弧度
    double heading = 0;
    if (gps_msg.heading_valid_) {
        // note: 由于RTK生产厂商协议并不相同，有些厂商在输出角度时会按照预定方案实现，进而导致输出角度定义并不相同，这边采用的千寻的RTK输出角度就是“北东地”方式。
        // 因此要将北东地坐标系转换为东北天坐标系
        /**
         *    x<北东地>(y<东北天>)
         *    ^        y 
         *    |    base|_x                    这边的话机器人姿态是不会变的，变得是在不同坐标系中的表示，这样就会发现机器人姿态在“北东地”、“东北天”两个坐标中的表示差90度
         *    |
         *    |--------> y<北东地>（x<东北天>）
         */
        heading = (90 - gps_msg.heading_) * math::kDEG2RAD;  // 北东地转到东北天
    }

    /// TWG 转到 TWB
    SE3 TBG(SO3::rotZ(antenna_angle * math::kDEG2RAD), Vec3d(antenna_pos[0], antenna_pos[1], 0));
    SE3 TGB = TBG.inverse();

    /// 若指明地图原点，则减去地图原点
    double x = utm_rtk.xy_[0] - map_origin[0];
    double y = utm_rtk.xy_[1] - map_origin[1];
    double z = utm_rtk.z_ - map_origin[2];
    SE3 TWG(SO3::rotZ(heading), Vec3d(x, y, z));
    SE3 TWB = TWG * TGB;

    gps_msg.utm_valid_ = true;
    gps_msg.utm_.xy_[0] = TWB.translation().x();
    gps_msg.utm_.xy_[1] = TWB.translation().y();
    gps_msg.utm_.z_ = TWB.translation().z();

    if (gps_msg.heading_valid_) {
        // 组装为带旋转的位姿
        gps_msg.utm_pose_ = TWB;
    } else {
        // 组装为仅有平移的SE3
        // 注意当安装偏移存在时，并不能实际推出车辆位姿
        gps_msg.utm_pose_ = SE3(SO3(), TWB.translation());
    }

    return true;
}

// 如果gps并未提供姿态的话，是没有办法计算map->base的
bool ConvertGps2UTMOnlyTrans(GNSS& gps_msg) {
    /// 经纬高转换为UTM
    UTMCoordinate utm_rtk;
    LatLon2UTM(gps_msg.lat_lon_alt_.head<2>(), utm_rtk);
    gps_msg.utm_valid_ = true;
    gps_msg.utm_.xy_ = utm_rtk.xy_;
    gps_msg.utm_.z_ = gps_msg.lat_lon_alt_[2];
    gps_msg.utm_pose_ = SE3(SO3(), Vec3d(gps_msg.utm_.xy_[0], gps_msg.utm_.xy_[1], gps_msg.utm_.z_));
    return true;
}

}  // namespace sad