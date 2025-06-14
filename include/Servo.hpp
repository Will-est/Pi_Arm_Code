
namespace Servo
{
    class servo
    {
        private:
            int id;
            int pin;
            int angle;

        public:
            servo(int id, int pin);
            void setAngle(int theta);
            int getAngle();
    }
}