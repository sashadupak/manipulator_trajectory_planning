# manipulator_trajectory_planning
## Описание
- Решение прямой и обратной задачи кинематики
- Планирование траектории движения
- Траекторное управление
- Моделирование процесса управления

## DH-параметры манипулятора
| Узел | a, cm | α, ° | d, cm | θ |
|:----:|:------:|:------------:|:------:|:-------:|
| 1 | 0 | 90 | 14 | θ1* |
| 2 | 12 | 0 | 0 | θ2* |
|3| 12 | 0 | 0 | θ3* |

## Прямая задача кинематики
Подставив DH пареметры манипулятора, получим три матрицы однородного преобразования:  

<img src="https://latex.codecogs.com/gif.latex?T_i=\begin{bmatrix}cos(\theta)&-sin(\theta)cos(\alpha)&sin(\theta)sin(\alpha)&a\cdot\cos(\theta)\\sin(\theta)&cos(\theta)cos(\alpha)&-cos(\theta)sin(\alpha)&a\cdot\sin(\alpha)\\0&sin(\alpha)&cos(\alpha)&d\\0&0&0&1\end{bmatrix}">  

Итоговую матрицу получаем последовательным перемножением:  

<img src="https://latex.codecogs.com/gif.latex?T=T_1\cdot\;T_2\cdot\;T_3=\begin{bmatrix}r_1_1&r_1_2&r_1_3&x\\r_2_1&r_2_2&r_2_3&y\\r_3_1&r_3_2&r_3_3&z\\0&0&0&1\end{bmatrix}">  
Определение углов Эйлера:  

Если <img src="https://latex.codecogs.com/gif.latex?r_3_3\neq\pm\1">:  

<img src="https://latex.codecogs.com/gif.latex?\theta=atan2(\pm\sqrt(1-r_3_3^2),r_3_3)">  

<img src="https://latex.codecogs.com/gif.latex?\phi=atan2(\pm\;r_2_3,\pm\;r_1_3)">  

<img src="https://latex.codecogs.com/gif.latex?\psi=atan2(\pm\;r_3_2,\mp\;r_3_1)">

## Обратная задача кинематики

## Планирование траектории
Гармоническая траектория основанная на тригонометрических функциях имеет
следующую математическую формулировку:
<img src="https://latex.codecogs.com/gif.latex?s(\theta)=R(1−cos(\theta))">, 
где R – радиус окружности.
В более общем случае, гармоническая траектория может быть определена как
<img src="https://latex.codecogs.com/gif.latex?q(t)=\frac{q_1-q_0}{2}(1-cos(\frac{t-t_0}{t_p}))+q_0">  

где q0, q1 – начальное и конечное значение угла поворота вала двигателя, t0, t1 - начальное и
конечное значение времени, t – текущее время, tp – время переходного процесса.
Продифференцировав представленное выражение по времени получим формулу для
угловой скорости вражения вала двигателя:  

<img src="https://latex.codecogs.com/gif.latex?\dot{q}(t)=\frac{\pi(q_1-q_0)}{2t_p}sin(\frac{\pi(t-t_0)}{t_p})">  

## Результаты

