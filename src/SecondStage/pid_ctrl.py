from typing import Union


class pidc:
    def __init__(self, kp: Union[float, int], ki: Union[float, int], kd: Union[float, int], max_total: int = 60, max_turn_angle: Union[int, float] = 30, reset: bool = False) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.__total_error = 0
        self.__output = 0

        self.prev_error = 0
        self.reset = reset
        self.max_total = max_total
        self.max_turn_angle = max_turn_angle
        self.sign = 1
        self.setpoint = 0

    @property
    def total_error(self):
        return self.__total_error

    @total_error.setter
    def total_error(self, val):
        val = abs(val)
        if val > self.max_total:
            val = 0

        self.__total_error = min(val, self.max_total) * self.sign

    @property
    def output(self):
        return self.__output

    @output.setter
    def output(self, val):
        val = abs(val)

        self.__output = round(min(val, self.max_turn_angle) * self.sign, 2)

    def calc_pid(self, error: Union[float, int]):
        if error == 0:
            return error

        self.sign = -1 if error < 0 else 1
        # error = self.setpoint - error
        self.total_error += abs(error)

        output = (
            error * self.kp +
            self.total_error * self.ki +
            (error - self.prev_error) * self.kd
        )

        self.prev_error = error
        self.output = output

        return self.output


if __name__ == "__main__":
    distpid = pidc(0.29, 0.001, 0.17, 150, 50, True) 
    print(distpid.calc_pid((200 // 1.3) - 130))
    # for x in range(5):
    #     for i in range(1, 300):
    #         keep_left = i - 130
    #         keep_right = 130 - i
    #         print(distpid.calc_pid(keep_left), end = ", ")

    #     print("\n\n")
