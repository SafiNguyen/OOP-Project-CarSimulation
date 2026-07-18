#ifndef UNITS_H
#define UNITS_H

// ============================================================================
// Units.h
// ----------------------------------------------------------------------------
// CONTRACT: Toan bo he thong vat ly noi bo (Vehicle, Road, Graph, pathfinding)
// chi lam viec voi don vi SI: MET (m), GIAY (s), M/S.
//
// km/h va km CHI duoc phep xuat hien o 2 "bien" (boundary) cua he thong:
//   1) Luc doc map JSON (mapload.cpp)      -> convert km/h, km  =>  m/s, m
//   2) Luc hien thi len UI cho nguoi dung  -> convert m/s, m    =>  km/h, km
//
// KHONG mot ham tinh toan vat ly nao (Vehicle::update, calculateCurrentSpeed,
// Road::getTravelTime, PathFindingStrategy...) duoc phep nhan/chia 3.6 hay
// 1000.0 truc tiep trong logic cua no. Neu can quy doi, GOI ham o day.
// ============================================================================
namespace Units {

    constexpr double KM_TO_M_FACTOR   = 1000.0;
    constexpr double KMH_TO_MPS_FACTOR = 3.6;

    constexpr double kmToM(double km) { return km * KM_TO_M_FACTOR; }
    constexpr double mToKm(double m)  { return m / KM_TO_M_FACTOR; }

    constexpr double kmhToMps(double kmh) { return kmh / KMH_TO_MPS_FACTOR; }
    constexpr double mpsToKmh(double mps) { return mps * KMH_TO_MPS_FACTOR; }

}

#endif