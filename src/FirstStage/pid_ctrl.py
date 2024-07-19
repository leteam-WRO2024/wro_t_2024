from typing import Union


class pidc:
    def __init__(self, kp: Union[float, int], ki: Union[float, int], kd: Union[float, int], reset: bool = False) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.__total_error = 0
        self.__output = 0

        self.prev_error = 0
        self.reset = reset
        self.max_total = 60
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

        self.__output = round(min(val, 30) * self.sign, 2)

    def calc_pid(self, error: Union[float, int]):
        if error == 0:
            return error

        self.sign = -1 if error < 0 else 1
        # error = self.setpoint - error
        self.total_error += error

        output = (
            error * self.kp +
            self.total_error * self.ki +
            (error - self.prev_error) * self.kd
        )

        self.prev_error = error
        self.output = output

        return self.output


if __name__ == "__main__":
    distpid = pidc(0.9, 0, 0.02, True)
    for x in range(5):
        for i in range(1, 35):
            keep_left = i - 35
            keep_right = 35 - i
            print(distpid.calc_pid(keep_right), end = ", ")

        print("\n\n")
