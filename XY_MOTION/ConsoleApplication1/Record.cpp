#include <iostream>
#include <string>
#include <queue>

using namespace std;

#define INIT_MUL_VAL 1
//machine_state_mode
#define Data_Receive_Mode 0
#define Communication_Mode 1
#define Move_Mode 2
#define Wait_Mode 3
#define Reset_Mode 5
//motor state
#define Motor_Idle 0
#define Motor_Operating 1
#define CW 0
#define CCW 1
//PWM state----
#define PWM_UNFINISHED 0
#define PWM_FINISHED 1
//line state
#define Y_LINE 0
#define X_LINE 1
#define DIAG_LINE 2

#define LIMIT_ 1

int G_code = -1; // g코드 명령어 종류 분류
int machine_state = 0; // 동작 순서
float mul = INIT_MUL_VAL; // 모터 속도 조절
float mul_value = 1;

double search_str(char key, queue<string> q);

void stepmotor_dir_set(double x, double y);
void Move_Motor(double x, double y);

void Motor_Rst(void);

void Check_PWM_State();
//void Reset_Point_Move(void);
//void Starting_Point_Move(void);

typedef struct
{
    double x;
    double y;
}pt_db;

typedef struct
{
    int x;
    int y;
}pt_int;

//Related Pysical distance-------------------
pt_db target_mm; // 100당 1mm, 목표 좌표
pt_db dir_considered_target_mm; // 100당 1mm, 방향을 고려한 목표 좌표
//Related PWM--------------------------------
pt_int pulse; // x축 모터 스텝수 임시 저장(실제 펄스가 일어나는 부분을 세는 부분으로 모터 구동의 심장박동을 측정한다)
pt_int target_pulse; // x축 모터 목표 스텝수// 구동을 위한 최종 심장박동 목표값
pt_int pwm_status; //x, y축 pwm종료 알림 신호, if==1  -> pwm 종료
//------------------------------------------
pt_db ellipse; // 타원 값 변수
pt_db current_mm;
pt_db Circle_Delta_mm;

pt_int Limit_start; // 리미트 스위치 on, off 변수


int x_pulse_ARR = 5250; // x축 모터 타이머 TIM4의 ARR
int y_pulse_ARR = 5250; // x축 모터 타이머 TIM4의 ARR

double ratio = 0; // "x의 목표 스텝수/y의 목표 스텝수"로 구한 기울기
double radius = 0; // G2, G3 반지름
int G2_reverse;
int G2_loop = 0;
double stack = 0;

int G5_loop = 0;
int G6_loop = 0;

int line_type;


int main()
{   
    Circle_Delta_mm.x = 0;
    Circle_Delta_mm.y = 0;
    ellipse.x = 1;
    ellipse.y = 1;
    current_mm.x = 0;
    current_mm.y = 0;
    pwm_status.x = PWM_UNFINISHED;
    pwm_status.y = PWM_UNFINISHED;
    Limit_start.x = 0;
    Limit_start.y = 0;

    
    double cutted_range_left = 0;
    double cutted_range_right = 0;

    queue<string> que_str;
    //원----
    que_str.push("G2 X18 Y9 I9");
    //que_str.push("G2 X-18 Y9 I-9");
    //clover-------
    //que_str.push("G2 X24 Y5 I12");
    //que_str.push("G2 X-24 Y5 I-12");
    //que_str.push("G3 X5 Y24 I12");
    //que_str.push("G3 X5 Y-24 I-12");
    //que_str.push("G2 X-24 Y5 I12");
    //que_str.push("G2 X24 Y5 I-12");
    //que_str.push("G3 X5 Y-24 I12");
    //que_str.push("G3 X5 Y24 I-12");
    //-------
    que_str.push("G0 X5");
    que_str.push("G1 X5 Y25");
    que_str.push("G1 X5");
    que_str.push("G1 X-5 Y-25");
    que_str.push("G1 X-5");
    que_str.push("G0 X12.5");
    que_str.push("G1 X-5 Y12.5");
    que_str.push("G1 X7.5 Y12.5");
    que_str.push("G1 X5");
    que_str.push("G1 X-7.5 Y-12.5");
    que_str.push("G1 X5 Y-12.5");
    que_str.push("G1 X-5");
    que_str.push("G0 X12.5");
    que_str.push("G1 Y25");
    que_str.push("G1 X5");
    que_str.push("G1 Y5");
    que_str.push("G1 X10");
    que_str.push("G1 Y-10");
    que_str.push("G2 X-5 Y5 I-5");
    que_str.push("G1 X-5");
    que_str.push("G1 Y-15");
    que_str.push("G1 X-5");
    que_str.push("G0 X5 Y20");
    que_str.push("G1 Y5");
    que_str.push("G1 X5");
    que_str.push("G1 Y-5");
    que_str.push("G1 X-5");
    que_str.push("G0 X12.5 Y-20");
    que_str.push("G1 X5 Y20");
    que_str.push("G1 X5");
    que_str.push("G1 X-3.5 Y-15");
    que_str.push("G1 X10");
    que_str.push("G1 X3.5 Y15");
    que_str.push("G1 X5");
    que_str.push("G1 X-5 Y-20");
    que_str.push("G1 X-20");
    que_str.push("G0 X30");
        
        
    machine_state = Communication_Mode;

    while (1)
    {
        switch (machine_state)
        {
        case Communication_Mode:
        {
            if (que_str.empty()) continue;

            double cutted_range = 0;

            G_code = (int)search_str('G', que_str); //G를 찾을경우 값을 반환
            if (G_code == 00 || G_code == 01)// 직선 이동일 경우
            {
                target_mm.x = search_str('X', que_str) * 100; //X를 찾을경우 그에대한 값을 반환
                target_mm.y = search_str('Y', que_str) * 100; //Y를 찾을경우 그에대한 값을 반환
                que_str.pop();
                machine_state = Move_Mode; //수신 받은 값을  설정 및 가공하기 위한 플래그 값
            }
            else if (G_code == 02 || G_code == 03)// 원호 이동일 경우
            {
                if (G_code == 02)
                {
                    target_mm.x = search_str('X', que_str) * 100;
                    ellipse.y = search_str('Y', que_str) * 100;

                    cutted_range = target_mm.x / 10;

                    //10의 자리 숫자까지는 전부 제거
                    if ((int)cutted_range % 100 != 0)
                    {
                        cutted_range = cutted_range - ((int)cutted_range % 100);
                    }

                    cutted_range_left = fabs(cutted_range);
                    cutted_range_right = fabs(target_mm.x - cutted_range);
                }
                else
                {
                    target_mm.y = search_str('Y', que_str) * 100;
                    ellipse.x = search_str('X', que_str) * 100;

                    cutted_range = target_mm.y / 10;

                    //10의 자리 숫자까지는 전부 제거
                    if ((int)cutted_range % 100 != 0)
                    {
                        cutted_range = cutted_range - ((int)cutted_range % 100);
                    }

                    cutted_range_left = fabs(cutted_range);
                    cutted_range_right = fabs(target_mm.y - cutted_range);
                }

                if (search_str('I', que_str) < 0)
                    G2_reverse = CCW; // ('-') :역회전 변수를 On
                else 
                    G2_reverse = CW; //('+') or (' ') : 역회전 변수를 Off

                radius = fabs(search_str('I', que_str) * 100); 

                

                que_str.pop();
                machine_state = Move_Mode;
            }
            /*else if (G_code == 05 || G_code == 06)
            {
                que_str.pop();
                machine_state = Move_Mode;
            }*/
            /*else if (G_code == 07)
            {
                mul_value = search_str('S', que_str);
                que_str.pop();
                machine_state = Reset_Mode;
            }*/
            break;
        }
        case Move_Mode:
        {
            if (G_code == 01 || G_code == 02 || G_code == 03)
            {
                mul = mul_value;
            }
            if (G_code == 00)
            {
                mul = 0.1;
                stepmotor_dir_set(target_mm.x, target_mm.y); //모터의 방향을 설정한다.
                Move_Motor(dir_considered_target_mm.x, dir_considered_target_mm.y);
                machine_state = Wait_Mode;
            }
            else if (G_code == 01)
            {
                stepmotor_dir_set(target_mm.x, target_mm.y); //모터의 방향을 설정한다.
                Move_Motor(dir_considered_target_mm.x, dir_considered_target_mm.y);
                machine_state = Wait_Mode;
            }
            else if (G_code == 02)
            {
                switch (G2_loop)//원호를 계속해서 그려줘야하기에 LOOP로 변수설정
                {

                // X-Calculate
                case 0:
                {
                    if (fabs(current_mm.x) < cutted_range_left)
                        Circle_Delta_mm.x = 10;// 10 == 0.1mm, 0.1mm 씩 계속 루프문 돌린다.
                    else if (fabs(current_mm.x) > cutted_range_right)
                        Circle_Delta_mm.x = 10;// 10 == 0.1mm, 0.1mm 씩 계속 루프문 돌린다.
                    else
                        Circle_Delta_mm.x = 100; // 100 == 1mm, 1mm 씩 계속 루프문 돌린다.

                    if (target_mm.x > 0)
                    {
                        current_mm.x += Circle_Delta_mm.x;
                        G2_loop = 1;
                        if (current_mm.x > target_mm.x) //point==설정, 설정보다 실제 움직인 값이 더 크면 종료
                        {
                            machine_state = Reset_Mode;
                        }
                    }
                    else if (target_mm.x < 0)
                    {
                        current_mm.x -= Circle_Delta_mm.x;
                        G2_loop = 1;
                        if (current_mm.x < target_mm.x)
                        {
                            machine_state = Reset_Mode;
                        }
                    }
                    break;
                }
                    
                // Y-Calculate
                case 1:
                {
                    double tmp = sqrt((radius * radius) - (fabs(current_mm.x) - radius) * (fabs(current_mm.x) - radius)) - current_mm.y;
                    Circle_Delta_mm.y = tmp;    //원호를 그리기 위해서 계속해서 위치값을 최신화 시키는 변수
                    stack += tmp - Circle_Delta_mm.y;// 최신화된 변수를 계속해서 합산하여 새로운 이동지점을 계속해서 만들어냄
                    if (stack >= 10)
                    {
                        stack -= 10;
                        Circle_Delta_mm.y += 10;
                    }

                    current_mm.y += Circle_Delta_mm.y;
                    if (G2_reverse == CW)
                        stepmotor_dir_set(Circle_Delta_mm.x, Circle_Delta_mm.y);//구한 y값을 방향 세팅
                    else 
                        stepmotor_dir_set(Circle_Delta_mm.x, -Circle_Delta_mm.y);

                    Move_Motor(Circle_Delta_mm.x, dir_considered_target_mm.y * ellipse.y / radius);
                    G2_loop = 2;
                    break;
                }
                case 2:
                {
                    Check_PWM_State();
                    if (line_type == Y_LINE && pwm_status.y == PWM_FINISHED)//y축 이동 종료
                    {
                        pwm_status.y = PWM_UNFINISHED;
                        G2_loop = 0;
                        break;
                    }
                    else if (line_type == X_LINE && pwm_status.x == PWM_FINISHED)//x축 이동 종료
                    {
                        pwm_status.x = PWM_UNFINISHED;
                        G2_loop = 0;
                        break;
                    }
                    else if (line_type == DIAG_LINE && pwm_status.x == PWM_FINISHED && pwm_status.y == PWM_FINISHED)//x,y축 이동 종료
                    {
                        pwm_status.x = PWM_UNFINISHED;
                        pwm_status.y = PWM_UNFINISHED;
                        G2_loop = 0;
                        break;
                    }

                    //-----------------PWM 기능 완료 후 지정한 지점으로 움직였다고 가정
                    if (target_pulse.x > 0 && target_pulse.y > 0)
                    {
                        pwm_status.x = PWM_FINISHED;
                        pwm_status.y = PWM_FINISHED;
                    }
                    //----------------------------------------------------------
                    
                    break;
                }
                }
            }
            else if (G_code == 03)
            {
                //원호를 계속해서 그려줘야하기에 LOOP로 변수설정
                switch (G2_loop)
                {
                case 0:
                {
                    if (current_mm.y < cutted_range_left)
                        Circle_Delta_mm.y = 10;// 10 == 0.1mm, 0.1mm 씩 계속 루프문 돌린다.
                    else if (current_mm.y > cutted_range_right)
                        Circle_Delta_mm.y = 10;// 10 == 0.1mm, 0.1mm 씩 계속 루프문 돌린다.
                    else
                        Circle_Delta_mm.y = 100; // 100 == 1mm, 1mm 씩 계속 루프문 돌린다.

                    if (target_mm.y > 0)
                    {
                        current_mm.y += Circle_Delta_mm.y;// 100 == 1mm, 1mm 씩 계속 루프문 돌린다.
                        G2_loop = 1;
                        if (current_mm.y > target_mm.y) //point==설정, 설정보다 실제 움직인 값이 더 크면 종료
                        {
                            machine_state = Reset_Mode;
                        }
                    }
                    else if (target_mm.y < 0)
                    {
                        current_mm.y -= Circle_Delta_mm.y;
                        G2_loop = 1;
                        if (current_mm.y < target_mm.y)
                        {
                            machine_state = Reset_Mode;
                        }
                    }
                    break;
                }
                case 1:
                {
                    double tmp = sqrt((radius * radius) - (fabs(current_mm.y) - radius) * (fabs(current_mm.y) - radius)) - current_mm.x;
                    Circle_Delta_mm.x = tmp;
                    stack += tmp - Circle_Delta_mm.x;
                    if (stack >= 10)
                    {
                        stack -= 10;
                        Circle_Delta_mm.x += 10;
                    }
                    current_mm.x += Circle_Delta_mm.x;
                    if (G2_reverse == CW)
                        stepmotor_dir_set(Circle_Delta_mm.x, Circle_Delta_mm.y);//구한 y값을 방향 세팅
                    else 
                        stepmotor_dir_set(-Circle_Delta_mm.x, Circle_Delta_mm.y);

                    Move_Motor(dir_considered_target_mm.x * ellipse.x / radius, Circle_Delta_mm.y);
                    G2_loop = 2;
                    break;
                } 
                case 2:
                {
                    if (line_type == Y_LINE && pwm_status.y == PWM_FINISHED)//y축 이동 종료
                    {
                        pwm_status.y = PWM_UNFINISHED;
                        G2_loop = 0;
                        break;
                    }
                    else if (line_type == X_LINE && pwm_status.x == PWM_FINISHED)//x축 이동 이동 종료
                    {
                        pwm_status.x = PWM_UNFINISHED;
                        G2_loop = 0;
                        break;
                    }
                    else if (line_type == 2 && pwm_status.x == PWM_FINISHED && pwm_status.y == PWM_FINISHED)//x,y축 이동 이동 종료
                    {
                        pwm_status.x = PWM_UNFINISHED;
                        pwm_status.y = PWM_UNFINISHED;
                        G2_loop = 0;
                        break;
                    }

                    //-----------------PWM 기능 완료 후 지정한 지점으로 움직였다고 가정
                    if (target_pulse.x > 0 && target_pulse.y > 0)
                    {
                        pwm_status.x = PWM_FINISHED;
                        pwm_status.y = PWM_FINISHED;
                    }
                    //----------------------------------------------------------
                    break;
                }
                    
                }
            }
            /*else if (G_code == 05)
            {
                Reset_Point_Move();
                machine_state = Reset_Mode;
            }
            else if (G_code == 06)
            {
                Starting_Point_Move();
                machine_state = Reset_Mode;
            }*/
            break;
        }
            
        case Wait_Mode:
        {
            if (line_type == Y_LINE && pwm_status.y == PWM_FINISHED)
            {
                pwm_status.y = PWM_UNFINISHED;

                machine_state = Reset_Mode;
                break;
            }
            else if (line_type == X_LINE && pwm_status.x == PWM_FINISHED)
            {
                pwm_status.x = PWM_UNFINISHED;

                machine_state = Reset_Mode;
                break;
            }
            else if (line_type == DIAG_LINE && pwm_status.x == PWM_FINISHED && pwm_status.y == PWM_FINISHED)
            {
                pwm_status.x = PWM_UNFINISHED;
                pwm_status.y = PWM_UNFINISHED;

                machine_state = Reset_Mode;
                break;
            }
            //-----------------PWM 기능 완료 후 지정한 지점으로 움직였다고 가정
            if (target_pulse.x > 0 ) pwm_status.x = PWM_FINISHED;
            if (target_pulse.y > 0) pwm_status.y = PWM_FINISHED;
            //----------------------------------------------------------
            break;
        }
            



        case Reset_Mode:
        {
            if (G_code == 02 || G_code == 03)
            {
                G2_reverse = CW;
                current_mm.x = 0;
                current_mm.y = 0;
                Circle_Delta_mm.x = 0;
                Circle_Delta_mm.y = 0;
                stack = 0;
                G2_loop = 0;
            }
            else if (G_code == 05 || G_code == 06)
            {
                G5_loop = 0;
                G6_loop = 0;
                Limit_start.x = 0;
                Limit_start.y = 0;
            }
            Motor_Rst();
            
            machine_state = Communication_Mode;

            G_code = -1;
            break;
        }
        }
            
    }
}

double search_str(char key, queue<string> q)
{
    string str = q.front();

    int index = str.find(key) + 1;

    
    string tmp_str;
    for (int i = index; i < str.size(); i++)
    {
        if (isdigit(str[i]) || str[i] == '.' || str[i] == '-')
            tmp_str += str[i];
        else
            break;
    }

    // Convert for strtod
    char const* c = tmp_str.data();
    return strtod(c, NULL);
}

void stepmotor_dir_set(double x, double y) //모터 방향 설정 함수
{
    if (x < 0)
    {
        dir_considered_target_mm.x = -x;
    }
    else if (x > 0)
    {
        dir_considered_target_mm.x = x;
    }
    else if (x == 0)
    {
        dir_considered_target_mm.x = 0;
    }
    //--------------------------------  
    if (y < 0)
    {
        dir_considered_target_mm.y = -y;
    }
    else if (y > 0)
    {
        dir_considered_target_mm.y = y;
    }
    else if (y == 0)
    {
        dir_considered_target_mm.y = 0;
    }
}

void Move_Motor(double x, double y)
{
    if (x == 0 && y != 0) //y로 이동하는 직선이라면
    {
        target_pulse.y = (int)(y / 1.125); //3200pulse당 360도_36mm//36mmX100=3600/1.125 = 3200pulse
        y_pulse_ARR = 5250;
        line_type = Y_LINE;
    }
    else if (x != 0 && y == 0)//x로 이동하는 직선이라면
    {
        target_pulse.x = (int)(x / 1.125);
        x_pulse_ARR = 5250;// 기울기 때문에 바뀔수 있기 때문에 고정시킴
        line_type = X_LINE;
    }
    else if (x != 0 && y != 0)// 대각선이라면
    {
        target_pulse.x = (int)(x / 1.125); // 방향을 고려한 x의 목표 mm단위를 pulse단위로 바꾼다.
        target_pulse.y = (int)(y / 1.125); // 방향을 고려한 y의 목표 mm단위를 pulse단위로 바꾼다
        ratio = ((double)target_pulse.x / (double)target_pulse.y); // "x의 목표 스텝수/y의 목표 스텝수"로 구한 기울기
        if (ratio > 1)
        {
            x_pulse_ARR = 5250;// 5250(TIM3-y의 ARR)을 기울기로 나눠 x-TIM4의 ARR을 설정할 값 
            y_pulse_ARR = (int)(5250 * ratio);
        }
        else if (ratio < 1)
        {
            x_pulse_ARR = (int)(5250 / ratio); // 5250(TIM3-y의 ARR)을 기울기로 나눠 x-TIM4의 ARR을 설정할 값 
            y_pulse_ARR = 5250;
        }
        else
        {
            x_pulse_ARR = 5250; // 5250(TIM3-y의 ARR)을 기울기로 나눠 x-TIM4의 ARR을 설정할 값 
            y_pulse_ARR = 5250;
        }


        if ((int)(y_pulse_ARR * mul) - 1 > 65536) //65536
        {
            pwm_status.y = PWM_FINISHED;
        }
        else
        {
        }


        if ((int)(x_pulse_ARR * mul) - 1 > 65536)
        {
            pwm_status.x = PWM_FINISHED;
        }
        else
        {
        }

        line_type = DIAG_LINE;
    }
}

void Motor_Rst(void)
{
    dir_considered_target_mm.x = 0;
    dir_considered_target_mm.y = 0;
    target_mm.x = 0;
    target_mm.y = 0;
    //------------------------------
    ellipse.x = 1;
    ellipse.y = 1;
    radius = 0;
    //------------------------------
    pwm_status.x = PWM_UNFINISHED;
    pwm_status.y = PWM_UNFINISHED;
    target_pulse.x = 0;
    target_pulse.y = 0;
    pulse.x = 0;
    pulse.y = 0;
}

void Check_PWM_State()
{

}

//void Starting_Point_Move(void)
//{
//    while (G6_loop != 8)
//    {
//        if (G6_loop == 0)
//        {
//            mul = 0.1;
//            stepmotor_dir_set(-100000, 0); //모터의 방향을 설정한다.
//            Move_Motor(dir_considered_target_mm.x, 0);
//            G6_loop = 1;
//        }
//        else if (G6_loop == 1)
//        {
//            while (Limit_start.x == 0) {}
//            Motor_Rst();
//            Limit_start.x = 0;
//            G6_loop = 2;
//        }
//        else if (G6_loop == 2)
//        {
//            stepmotor_dir_set(1500, 0); //모터의 방향을 설정한다.
//            Move_Motor(dir_considered_target_mm.x, 0);
//            G6_loop = 3;
//        }
//        else if (G6_loop == 3)
//        {
//            while (pwm_status.x == PWM_UNFINISHED) {}
//            Motor_Rst();
//            pwm_status.x = PWM_UNFINISHED;
//            G6_loop = 4;
//        }
//        else if (G6_loop == 4)
//        {
//            stepmotor_dir_set(0, -100000); //모터의 방향을 설정한다.
//            Move_Motor(0, dir_considered_target_mm.y);
//            G6_loop = 5;
//        }
//        else if (G6_loop == 5)
//        {
//            while (Limit_start.y == 0) {}
//            Motor_Rst();
//            Limit_start.y = 0;
//            G6_loop = 6;
//        }
//        else if (G6_loop == 6)
//        {
//            stepmotor_dir_set(0, 1500); //모터의 방향을 설정한다.
//            Move_Motor(0, dir_considered_target_mm.y);
//            G6_loop = 7;
//        }
//        else if (G6_loop == 7)
//        {
//            while (pwm_status.y == PWM_UNFINISHED) {}
//            Motor_Rst();
//            pwm_status.y = PWM_UNFINISHED;
//            G6_loop = 8;
//        }
//    }
//}
//
//void Reset_Point_Move(void)
//{
//    while (G5_loop != 8)
//    {
//        if (G5_loop == 0)
//        {
//            mul = 0.1;
//            stepmotor_dir_set(-100000, 0); //모터의 방향을 설정한다.
//            Move_Motor(dir_considered_target_mm.x, 0);
//            G5_loop = 1;
//        }
//        else if (G5_loop == 1)
//        {
//            while (Limit_start.x == 0) {}
//            Motor_Rst();
//            Limit_start.y = 0;
//            G5_loop = 2;
//        }
//        else if (G5_loop == 2)
//        {
//            stepmotor_dir_set(8000, 0); //모터의 방향을 설정한다.
//            Move_Motor(dir_considered_target_mm.x, 0);
//            G5_loop = 3;
//        }
//        else if (G5_loop == 3)
//        {
//            while (pwm_status.x == 0) {}
//            Motor_Rst();
//            pwm_status.x = 0;
//            G5_loop = 4;
//        }
//        else if (G5_loop == 4)
//        {
//            stepmotor_dir_set(0, -100000); //모터의 방향을 설정한다.
//            Move_Motor(0, dir_considered_target_mm.y);
//            G5_loop = 5;
//        }
//        else if (G5_loop == 5)
//        {
//            while (Limit_start.y == 0) {}
//            Motor_Rst();
//            Limit_start.y = 0;
//            G5_loop = 6;
//        }
//        else if (G5_loop == 6)
//        {
//            stepmotor_dir_set(0, 6800); //모터의 방향을 설정한다.
//            Move_Motor(0, dir_considered_target_mm.y);
//            G5_loop = 7;
//        }
//        else if (G5_loop == 7)
//        {
//            while (pwm_status.y == 0) {}
//            Motor_Rst();
//            pwm_status.y = 0;
//            G5_loop = 8;
//        }
//    }
//}
//
//
