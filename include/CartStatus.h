
class CartStatus
{
private:
    float _fLeftTrackVelocity;
    float _fRightTrackVelocity;
    float _fAngle;
public:
    void vSetLeftTrackVelo(float fVelo);
    void vSetRightTrackVelo(float fVelo);
    void vSetAngle(float fAngle);
    float fGetLeftTrackVelo();
    float fGetRightTrackVelo();
    float fGetAngle();
};