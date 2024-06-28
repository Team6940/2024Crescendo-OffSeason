package frc.robot.Library;


public class NumberLimiter {
    public static double Limit(double _Min,double _Max,double _NumberToLimit)
    {
        double _LimitedNumber=_NumberToLimit;
        _LimitedNumber=Math.min(_Max,_LimitedNumber);
        _LimitedNumber=Math.max(_Min,_LimitedNumber);
        return _LimitedNumber;
    }
}
