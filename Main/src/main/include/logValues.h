#include <frc/smartdashboard/SmartDashboard.h>
#include <string>

void logSwerveNumber (std::string message, double FL, double FR, double BL, double BR)
{
    std::string mFL = "Front Left" + message;
    std::string mFR = "Front Right" + message;
    std::string mBL = "Back Left" + message;
    std::string mBR = "Back Right" + message;
    
    frc::SmartDashboard::PutNumber(mFL.c_str(), FL);
    frc::SmartDashboard::PutNumber(mFR.c_str(), FR);
    frc::SmartDashboard::PutNumber(mBL.c_str(), BL);
    frc::SmartDashboard::PutNumber(mBR.c_str(), BR);
}

void logSwerveString (std::string message, std::string FL, std::string FR, std::string BL, std::string BR)
{
    std::string mFL = "Front Left " + message;
    std::string mFR = "Front Right " + message;
    std::string mBL = "Back Left " + message;
    std::string mBR = "Back Right " + message;
    
    frc::SmartDashboard::PutString(mFL.c_str(), FL.c_str());
    frc::SmartDashboard::PutString(mFR.c_str(), FR.c_str());
    frc::SmartDashboard::PutString(mBL.c_str(), BL.c_str());
    frc::SmartDashboard::PutString(mBR.c_str(), BR.c_str());
}

