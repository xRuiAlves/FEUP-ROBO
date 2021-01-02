from abc import ABC, abstractmethod
import random

class AbstractInjector(ABC):
    def __init__(self, wrappee):
        if wrappee and not isinstance(wrappee, AbstractInjector):
            raise Exception('Wrappee not a subclass of AbstractInjector')

        self.__wrappee = wrappee

    @abstractmethod
    def _do_mutate_message(self, msg):
        return msg

    def mutate_message(self, message):
        """
        Takes as input the message received from the topic.
        Returns the message with the respective mutations.
        The default is just retransmitting the same data.
        """
        if self.__wrappee:
            message = self.__wrappee.mutate_message(message)

        return self._do_mutate_message(message)
    
    # @abstractmethod
    def _print_self(self):
        print(self.__class__.__name__, vars(self))

    def print_pipeline(self):
        if self.__wrappee:
            self.__wrappee.print_pipeline()

        self._print_self()

class FixedInjector(AbstractInjector):
    def __init__(self, wrappee, fixed_value):
        super().__init__(wrappee)
        self.__fixed_value = fixed_value

    def _do_mutate_message(self, msg):
        msg.ranges = [self.__fixed_value] * len(msg.ranges)
        return msg

class ScaleInjector(AbstractInjector):
    def __init__(self, wrappee, scale_factor):
        super().__init__(wrappee)
        self.__scale_factor = scale_factor

    def _do_mutate_message(self, msg):
        msg.ranges = [(self.__scale_factor * val) for val in msg.ranges]
        return msg

class RandomInjector(AbstractInjector):
    def __init__(self, wrappee):
        super().__init__(wrappee)

    def _do_mutate_message(self, msg):
        msg.ranges = [random.uniform(0, 1) for _ in msg.ranges]
        return msg

class NullInjector(AbstractInjector):
    def __init__(self, wrappee):
        super().__init__(wrappee)

    def _do_mutate_message(self, msg):
        msg.ranges = [0.0] * len(msg.ranges)
        return msg

class AddConstantInjector(AbstractInjector):
    def __init__(self, wrappee, constant):
        super().__init__(wrappee)
        self.__constant = constant

    def _do_mutate_message(self, msg):
        msg.ranges = [(self.__constant + val) for val in msg.ranges]
        return msg
