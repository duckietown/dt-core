from easy_regression.processor_interface import ProcessorInterface

__all__ = ['IdentityProcessor']


class IdentityProcessor(ProcessorInterface):

    def process_log(self, bag_in, prefix_in, bag_out, prefix_out):  #@UnusedVariable
        # TODO
        for topic, msg, _t in bag_in.read_messages():
            bag_out.write(topic, msg)
