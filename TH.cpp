#include <bits/stdc++.h>
using namespace std;

class DoThi {
public:
    map<int, vector<int>> dSachKe;
    map<int, map<int, int>> chiPhi;

    void themCanh(int dDau, int dCuoi, int cost) {
        dSachKe[dDau].push_back(dCuoi);
        dSachKe[dCuoi].push_back(dDau);
        chiPhi[dDau][dCuoi] = cost;
        chiPhi[dCuoi][dDau] = cost;
    }

    vector<int> DSachKe1Dinh(int dinh) {
        return dSachKe[dinh];
    }

    int layChiPhi(int dDau, int dCuoi) {
        return chiPhi[dDau][dCuoi];
    }
};

int heuristic(int current, int goal) {
    return abs(current - goal);
}

// Hàm A* tìm đường đi từ đỉnh start đến đỉnh goal
vector<int> timDuongDiAStar(DoThi& doThi, int batDau, int mucTieu) {
    map<int, int> luug;
    map<int, int> luuf;
    map<int, int> cameFrom;

    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> openSet;
    openSet.push({0, batDau});
    luug[batDau] = 0;
    luuf[batDau] = heuristic(batDau, mucTieu);

    while (!openSet.empty()) {
        int current = openSet.top().second;  // Đỉnh có luuf thấp nhất
        openSet.pop();

        if (current == mucTieu) {
            vector<int> duongDi;
            while (cameFrom.find(current) != cameFrom.end()) {
                duongDi.push_back(current);
                current = cameFrom[current];
            }
            duongDi.push_back(batDau);
            reverse(duongDi.begin(), duongDi.end());
            return duongDi;
        }

        for (int keBen : doThi.DSachKe1Dinh(current)) {
            int DiemCua_g_TamThoi = luug[current] + doThi.layChiPhi(current, keBen);
            if (DiemCua_g_TamThoi < luug[keBen] || luug.find(keBen) == luug.end()) {
                cameFrom[keBen] = current;
                luug[keBen] = DiemCua_g_TamThoi;
                luuf[keBen] = luug[keBen] + heuristic(keBen, mucTieu);
                openSet.push({luuf[keBen], keBen});
            }
        }
    }

    return {};
}

bool minimax(DoThi& doThi, int hienTai, int mucTieu, map<int, bool>& daTham, vector<int>& duongDi, bool dgditoiuu) {
    if (hienTai == mucTieu) {
        duongDi.push_back(mucTieu);
        return true;
    }

    daTham[hienTai] = true;

    if (dgditoiuu) {
        int dinhTotNhat = -1;
        for (int keBen : doThi.DSachKe1Dinh(hienTai)) {
            if (!daTham[keBen]) {
                vector<int> duongDiPhu;
                if (minimax(doThi, keBen, mucTieu, daTham, duongDiPhu, false)) {
                    dinhTotNhat = keBen;
                    duongDi = duongDiPhu;
                    break;
                }
            }
        }
        if (dinhTotNhat != -1) {
            duongDi.insert(duongDi.begin(), hienTai);
            daTham[hienTai] = false;
            return true;
        }
    } else {
        for (int keBen : doThi.DSachKe1Dinh(hienTai)) {
            if (!daTham[keBen]) {
                vector<int> duongDiPhu;
                if (minimax(doThi, keBen, mucTieu, daTham, duongDiPhu, true)) {
                    duongDi = duongDiPhu;
                    duongDi.insert(duongDi.begin(), hienTai);
                    daTham[hienTai] = false;
                    return true;
                }
            }
        }
    }

    daTham[hienTai] = false;
    return false;
}

int tinhChiPhi(const vector<int>& duongDi, DoThi& doThi) {
    int chiPhiT = 0;
    for (size_t i = 1; i < duongDi.size(); ++i) {
        chiPhiT += doThi.layChiPhi(duongDi[i - 1], duongDi[i]);
    }
    return chiPhiT;
}

int main() {
    DoThi doThi;
    int soDinh, soCanh;
    cout << "Nhap so dinh: "; cin >> soDinh;
    cout << "Nhap so canh: "; cin >> soCanh;
    cout << "Nhap cac canh (dinh bat dau, dinh ket thuc, chi phi):\n";
    for (int i = 0; i < soCanh; ++i) {
        int diemDau, diemCuoi, chiPhiCanh;
        cin >> diemDau >> diemCuoi >> chiPhiCanh;
        doThi.themCanh(diemDau, diemCuoi, chiPhiCanh);
    }
    int batDau, mucTieu;
    cout << "Nhap dinh bat dau: "; cin >> batDau;
    cout << "Nhap dinh dich: "; cin >> mucTieu;

    vector<int> duongDiAStar = timDuongDiAStar(doThi, batDau, mucTieu);
    if (!duongDiAStar.empty()) {
        cout << "Duong di A* (duong di ngan nhat): ";
        for (int dinh : duongDiAStar) {
            cout << dinh << " ";
        }
        cout << endl;
        cout << "Chi phi duong di A*: " << tinhChiPhi(duongDiAStar, doThi) << endl;
    } else {
        cout << "Khong tim thay duong di A*!" << endl;
    }

    vector<int> duongDiMinimax;
    map<int, bool> daTham;
    bool timThay = minimax(doThi, batDau, mucTieu, daTham, duongDiMinimax, true);
    if (timThay) {
        cout << "Duong di Minimax (duong di toi uu): ";
        for (int dinh : duongDiMinimax) {
            cout << dinh << " ";
        }
        cout << endl;
        cout << "Chi phi duong di Minimax: " << tinhChiPhi(duongDiMinimax, doThi) << endl;
    } else {
        cout << "Khong tim thay duong di Minimax!" << endl;
    }
    return 0;
}