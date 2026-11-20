// ks_import.h - read KS-engine program banks (.syx) and turn them into Jamiro programs.
//
// SPDX-License-Identifier: MIT
//
// Jamiro ships without any factory sounds. At start-up it looks for .syx files in
//   <plugin folder>/Content/Banks/   (and in Content/ itself)
// and adds every program it finds to the preset list, in file-name order:
// first file = programs 100.., second file = 200.., and so on.
//
// Understood formats (Novation SysEx, manufacturer 00 20 29, device type 01):
//   * X-Station program dumps (device 42h, 270-byte messages): the 128-byte KS
//     program block plus the X-Station extension (17 waveforms, 32 LFO shapes,
//     filter type, mixer level modulation, 16-character name).
//   * KS4/KS5/KS Rack/K-Station program and program-pair dumps (device 41h/42h,
//     128-byte blocks without name): the K-Station subset.
// The byte layout of the 128-byte block is Novation's published "PROGRAM DATA
// BLOCK" table (K-Station manual, appendix A-9..A-11).
//
// Runs in the plugin constructor only (allocates, reads files).

#ifndef JAMIRO_KS_IMPORT_H
#define JAMIRO_KS_IMPORT_H

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#if defined(_WIN32)
  #include <windows.h>
#else
  #include <dirent.h>
#endif

#include "jamiro_trims.h"

namespace ksimport {

struct ImportedProgram { std::string name; std::vector<float> values; };

inline std::vector<std::string> listSyx(const std::string& dir)
{
    std::vector<std::string> out;
    auto isSyx = [](const std::string& n) {
        if (n.size() < 5 || n[0] == '.') return false;
        std::string e = n.substr(n.size() - 4);
        for (char& c : e) c = (char)std::tolower((unsigned char)c);
        return e == ".syx";
    };
#if defined(_WIN32)
    WIN32_FIND_DATAA fd;
    HANDLE h = FindFirstFileA((dir + "/*").c_str(), &fd);
    if (h == INVALID_HANDLE_VALUE) return out;
    do { if (!(fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) && isSyx(fd.cFileName)) out.push_back(dir + "/" + fd.cFileName); }
    while (FindNextFileA(h, &fd));
    FindClose(h);
#else
    if (DIR* d = opendir(dir.c_str()))
    {
        while (dirent* e = readdir(d)) if (isSyx(e->d_name)) out.push_back(dir + "/" + e->d_name);
        closedir(d);
    }
#endif
    std::sort(out.begin(), out.end());
    return out;
}

inline uint32_t crc32(const uint8_t* p, size_t n)
{
    uint32_t c = 0xFFFFFFFFu;
    for (size_t i = 0; i < n; ++i)
    {
        c ^= p[i];
        for (int k = 0; k < 8; ++k) c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1u)));
    }
    return ~c;
}

inline float trimFor(const uint8_t* block)
{
    const uint32_t crc = crc32(block, 128);
    for (const JamiroTrim& t : kJamiroTrims) if (t.crc == crc) return t.db;
    return 0.0f;
}

template <typename T> T clampv(T v, T lo, T hi) { return v < lo ? lo : (v > hi ? hi : v); }

// Fill `v` (plain values, parameter order) from one 128-byte block `x` and the optional
// X-Station extension `ext` (nullptr for K-Station/KS data).
inline void convertBlock(const uint8_t* x, const uint8_t* ext, float* v)
{
    auto b = [x](int i) { return (float)((int)x[i] - 64); };
    auto put = [v](int idx, float val) { v[idx] = val; };
    put(kMixLevel, (float)clampv((int)x[125], 0, 30));
    put(kVoiceMode, (float)((x[0] >> 3) & 3));
    put(kUnison, (float)((x[0] & 7) + 1));
    put(kFSlope, (float)((x[0] >> 5) & 1));
    put(kUniDetune, x[1]); put(kDrift, x[2]); put(kPorta, x[3]);
    put(kBendRange, clampv(std::fabs(b(9)), 0.0f, 12.0f));
    put(kSync, (float)((x[6] >> 6) & 1));
    for (int o = 0; o < 3; ++o)
    {
        const int p = kO1Wave + o * kOscStride, base = 7 + 8 * o;
        put(p + fWave, ext ? (float)clampv((int)ext[o], 0, 16) : (float)((x[5] >> (2 * o)) & 3));
        put(p + fOct, (float)(((x[6] >> (2 * o)) & 3) - 1));
        put(p + fSemi, clampv(b(base), -12.0f, 12.0f));
        put(p + fCent, clampv(b(base + 1), -50.0f, 50.0f));
        put(p + fLfo, b(base + 3)); put(p + fEnv, b(base + 4)); put(p + fPw, b(base + 5));
        put(p + fPwLfo, b(base + 6)); put(p + fPwEnv, b(base + 7));
    }
    put(kO1Lvl, x[37]); put(kO2Lvl, x[38]); put(kO3Lvl, x[39]); put(kNoiseLvl, x[40]); put(kRingLvl, x[41]);
    put(kFmLvl, x[56]); put(kFmEnv, b(57)); put(kFmAtk, x[59]); put(kFmDec, x[60]);
    put(kFType, ext ? (float)clampv((int)ext[12], 0, 2) : 0.0f);
    put(kCutoff, x[46]); put(kReso, x[44]); put(kDrive, x[43]); put(kQNorm, x[45]); put(kKeyTrack, x[47]);
    put(kFEnv, b(52)); put(kFLfo, b(51));
    put(kAAtk, x[62]); put(kADec, x[63]); put(kASus, x[64]); put(kARel, x[65]); put(kAVel, b(61));
    put(kMAtk, x[67]); put(kMDec, x[68]); put(kMSus, x[69]); put(kMRel, x[70]); put(kMVel, b(66));
    // K-Station packs only 4 LFO shapes (tri, saw, square, S/H) into byte 78.
    static const int kKsLfo[4] = { 1, 2, 3, 4 };
    put(kL1Wave, (float)(ext ? lfoFromXStation(ext[3]) : kKsLfo[(x[78] >> 2) & 3]));
    put(kL2Wave, (float)(ext ? lfoFromXStation(ext[4]) : kKsLfo[(x[78] >> 4) & 3]));
    put(kL1Speed, x[72]); put(kL1Delay, x[74]); put(kL1Sync, (float)clampv((int)x[73], 0, 34)); put(kL1Key, (float)((x[79] >> 1) & 1));
    put(kL2Speed, x[75]); put(kL2Delay, x[77]); put(kL2Sync, (float)clampv((int)x[76], 0, 34)); put(kL2Key, (float)((x[79] >> 4) & 1));
    put(kL1Common, (float)((x[79] >> 2) & 1)); put(kL2Common, (float)((x[79] >> 5) & 1));   // packed parameter 6: lock
    put(kMwVib, b(34)); put(kMwFilt, b(48)); put(kMwFLfo, b(53)); put(kAtVib, b(35)); put(kAtFilt, b(49));
    put(kArpOn, (float)((x[88] >> 2) & 1)); put(kArpMode, (float)clampv((int)x[87], 0, 6));
    put(kArpOct, (float)((x[88] & 3) + 1)); put(kArpRate, (float)clampv((int)x[85], 0, 15));
    put(kArpGate, x[86]); put(kArpLatch, (float)((x[88] >> 4) & 1));
    put(kArpTempo, (float)clampv((int)x[84] + 64, 64, 191));
    put(kDist, x[97]); put(kEqLevel, clampv(b(92), -64.0f, 63.0f)); put(kEqFreq, x[93]);
    put(kChSend, x[110]); put(kChType, (float)((x[121] >> 3) & 1)); put(kChRate, x[112]); put(kChDepth, x[115]); put(kChFb, b(114));
    put(kDlSend, x[100]); put(kDlTime, x[102]); put(kDlSync, (float)clampv((int)x[103], 0, 19)); put(kDlFb, x[104]); put(kDlWidth, x[105]);
    put(kRvSend, x[107]); put(kRvType, (float)clampv(x[121] & 7, 0, 5)); put(kRvDecay, x[109]);
    put(kPan, b(117)); put(kPanDepth, x[120]); put(kPanRate, x[118]);
    if (ext)
    {
        put(kMixL1O1, clampv((float)ext[13] - 64.0f, -64.0f, 63.0f));
        put(kMixL2O1, clampv((float)ext[14] - 64.0f, -64.0f, 63.0f));
        put(kMixL2O2, clampv((float)ext[15] - 64.0f, -64.0f, 63.0f));
        put(kMixAdO3, clampv((float)ext[16] - 64.0f, -64.0f, 63.0f));
        put(kFiltVel, clampv((float)ext[17] - 64.0f, -64.0f, 63.0f));
        put(kL1Uni, (float)(ext[5] & 1)); put(kL2Uni, (float)(ext[6] & 1));
        put(kL1Phase, (float)clampv((int)ext[7], 0, 127)); put(kL2Phase, (float)clampv((int)ext[8], 0, 127));
    }
    put(kVolume, clampv(-6.0f + trimFor(x), -40.0f, 6.0f));
}

// Parse one .syx file; append its programs. `defaults` holds every parameter's default.
inline void importFile(const std::vector<uint8_t>& data, const std::vector<float>& defaults,
                       int& number, std::vector<ImportedProgram>& out)
{
    size_t i = 0;
    while (i < data.size())
    {
        if (data[i] != 0xF0) { ++i; continue; }
        size_t e = i + 1;
        while (e < data.size() && data[e] != 0xF7) ++e;
        if (e >= data.size()) break;
        const uint8_t* m = data.data() + i;
        const size_t len = e - i + 1;
        i = e + 1;
        if (len < 13 + 128 + 1 || m[1] != 0x00 || m[2] != 0x20 || m[3] != 0x29 || m[4] != 0x01) continue;
        if (m[5] != 0x41 && m[5] != 0x42) continue;
        const uint8_t type = m[7];
        if (type > 0x02) continue;   // 0 current sound, 1 program, 2 program pair
        const size_t payload = len - 14;
        // X-Station: one 128-byte block + 128-byte extension (a KS pair dump is type 2 instead).
        const bool xstation = m[5] == 0x42 && type != 0x02 && payload >= 256;
        const int blocks = xstation ? 1 : (int)(payload / 128);
        for (int k = 0; k < blocks && k < 2; ++k)
        {
            const uint8_t* x = m + 13 + 128 * k;
            const uint8_t* ext = xstation ? x + 128 : nullptr;
            ImportedProgram p;
            p.values = defaults;
            convertBlock(x, ext, p.values.data());
            std::string name;
            if (ext)
                for (int c = 0; c < 16; ++c) { const char ch = (char)ext[23 + c]; name += (ch >= 32 && ch < 127) ? ch : ' '; }
            while (!name.empty() && name.back() == ' ') name.pop_back();
            if (name.empty()) name = "Program";
            p.name = std::to_string(number++) + " " + name;
            out.push_back(std::move(p));
        }
    }
}

inline std::vector<ImportedProgram> importAll(const std::vector<float>& defaults)
{
    std::vector<ImportedProgram> out;
    std::vector<std::string> files = listSyx(mpc::contentPath("Banks"));
    const std::vector<std::string> top = listSyx(mpc::contentFolder());
    files.insert(files.end(), top.begin(), top.end());
    int bank = 1;
    for (const std::string& f : files)
    {
        std::vector<uint8_t> data;
        if (!mpc::readFile(f, data)) continue;
        int number = 100 * bank;
        const size_t before = out.size();
        importFile(data, defaults, number, out);
        if (out.size() > before) ++bank;
    }
    return out;
}

} // namespace ksimport

#endif // JAMIRO_KS_IMPORT_H
