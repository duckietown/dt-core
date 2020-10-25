import os
import shutil

import duckietown_utils as dtu
from easy_algo import get_easy_algo_db
from easy_algo.algo_db import name_from_spec
from easy_logs import get_local_bag_file
from easy_logs.app_with_logs import download_if_necessary
from easy_logs.logs_structure import PhysicalLog
from easy_regression.processor_interface import ProcessorUtilsInterface, ProcessorInterface
import numpy as np
import rosbag
from std_msgs.msg import Float64, Int64, Float64MultiArray, MultiArrayDimension, MultiArrayLayout


class ProcessorUtils(ProcessorUtilsInterface):
    @dtu.contract(log=PhysicalLog)
    def __init__(self, log, bag_out):
        self.log = log
        self.bag_out = bag_out

    def write_stat(self, name, value, t=None, prefix=()):
        if isinstance(value, dict):
            for k, v in list(value.items()):
                self.write_stat(k, v, t=t, prefix=prefix + (name,))
            return
        else:
            msg = ros_from_misc(name, value, t)
            complete = prefix + (name,)
            topic = "/".join(complete)
            #             print('%s = %s' % (topic, msg))
            self.bag_out.write(topic, msg, t=t)

    def get_log(self):
        return self.log


def interpret_ros(msg):
    return np.array(msg.data)


def ros_from_misc(name, value, t):
    if isinstance(value, float):
        return Float64(value)
    elif isinstance(value, int):
        return Int64(value)
    elif isinstance(value, np.ndarray):
        msg = ros_from_np_array(value)
        #         print('%s -> %s' % (value, msg))
        return msg
    elif isinstance(value, list):
        data = np.array(value)
        return ros_from_np_array(data)
    else:
        m = 'Could not find a way to write "%s" in ROS (%s)' % (name, dtu.describe_type(value))
        m += "\n" + dtu.indent(str(value), "> ")
        raise ValueError(m)


def ros_from_np_array(data):
    if data.shape == ():
        msg = "I do not know how to convert this: \n%s\n%s" % (data.dtype, data)
        raise NotImplementedError(msg)
    dims = []
    for i, size in enumerate(data.shape):
        label = "dim%d" % i
        stride = 0
        dims.append(MultiArrayDimension(label=label, size=size, stride=stride))
    layout = MultiArrayLayout(dim=dims)

    d = list(data.flatten())
    msg = Float64MultiArray(data=d, layout=layout)
    return msg


@dtu.contract(log=PhysicalLog)
def process_one_dynamic(context, bag_filename, t0, t1, processors, log_out, log, delete, tmpdir):
    dtu.logger.info("process_one_dynamic()")
    dtu.logger.info("   input: %s" % bag_filename)
    dtu.logger.info("   processors: %s" % processors)
    dtu.logger.info("   out: %s" % log_out)
    dtu.logger.info("   t0 t1: %s %s" % (t0, t1))

    dtu.d8n_make_sure_dir_exists(log_out)

    tmpfiles = []

    def get_tmp_bag():
        i = len(tmpfiles)
        f = os.path.join(tmpdir, "process_one_dynamic-tmp%02d.bag" % i)
        tmpfiles.append(f)
        return f

    tmpfiles = []

    bag = rosbag.Bag(bag_filename)
    t0_absolute = bag.get_start_time() + t0
    t1_absolute = bag.get_start_time() + t1

    for i, processor_entry in enumerate(processors):
        processor_name = name_from_spec(processor_entry.processor)
        prefix_in = processor_entry.prefix_in
        prefix_out = processor_entry.prefix_out
        tmp = get_tmp_bag()

        bag_filename = context.comp(
            process_one_processor,
            processor_entry.processor,
            prefix_in,
            prefix_out,
            bag_filename,
            tmp,
            t0_absolute,
            t1_absolute,
            log,
            job_id="process-%d-%s" % (i, processor_name),
        )

    final = context.comp(finalize, bag_filename, log_out, processors, tmpfiles, delete)
    return final


def finalize(bag_filename, log_out, processors, tmpfiles, delete):
    dtu.logger.info("Creating output file %s" % log_out)
    if not processors:
        # just create symlink
        dtu.logger.info("(Just creating symlink, because there " "was no processing done.)")
        os.symlink(os.path.realpath(bag_filename), log_out)
    else:
        try:
            shutil.copy(bag_filename, log_out)
        except:
            dtu.logger.error("Could not create %s" % log_out)
    dtu.logger.info("I created %s" % log_out)

    if delete:
        for f in tmpfiles:
            if os.path.exists(f):
                dtu.logger.info(" deleting %s" % f)
                os.unlink(f)
    return log_out


def process_one_processor(
    processor_name_or_spec,
    prefix_in,
    prefix_out,
    bag_filename,
    next_bag_filename,
    t0_absolute,
    t1_absolute,
    log,
):
    dtu.DuckietownConstants.show_timeit_benchmarks = True

    easy_algo_db = get_easy_algo_db()
    processor = easy_algo_db.create_instance(ProcessorInterface.FAMILY, processor_name_or_spec)

    dtu.logger.info("in: bag_filename: %s" % bag_filename)
    if not os.path.exists(bag_filename):
        msg = "File does not exist: %s" % bag_filename
        raise ValueError(msg)
    dtu.logger.info("out: next_bag_filename: %s" % next_bag_filename)
    dtu.logger.info("t0_absolute: %s" % t0_absolute)
    dtu.logger.info("t1_absolute: %s" % t1_absolute)

    log = download_if_necessary(log)
    filename = get_local_bag_file(log)
    original_bag = rosbag.Bag(filename)
    bag_absolute_t0_ref = original_bag.get_start_time()
    original_bag.close()

    dtu.d8n_make_sure_dir_exists(next_bag_filename)
    out_bag = rosbag.Bag(next_bag_filename, "w")

    bag0 = rosbag.Bag(bag_filename)
    t0_rel = t0_absolute - bag0.get_start_time()
    t1_rel = t1_absolute - bag0.get_start_time()

    in_bag = dtu.BagReadProxy(bag0, t0_rel, t1_rel, bag_absolute_t0_ref=bag_absolute_t0_ref)

    utils = ProcessorUtils(bag_out=out_bag, log=log)
    processor.process_log(in_bag, prefix_in, out_bag, prefix_out, utils)
    in_bag.close()
    # also copy the other messages

    in_bag1 = rosbag.Bag(bag_filename)
    for topic, msg, t in in_bag1.read_messages(raw=True):
        out_bag.write(topic, msg, t, raw=True)
    in_bag1.close()

    out_bag.close()

    #     r = rosbag.Bag(next_bag_filename)
    #     for topic, msg, t in r.read_messages(raw=True):
    #         pass
    #     r.close()

    #     cmd = ['rosbag', 'fix', next_bag_filename, next_bag_filename]
    #     dtu.system_cmd_result(cwd='.', cmd=cmd, raise_on_error=True, display_stdout=True,
    #                           display_stderr=True)
    #     cmd = ['rosbag', 'reindex', next_bag_filename]
    #     dtu.system_cmd_result(cwd='.', cmd=cmd, raise_on_error=True, display_stdout=True,
    #                           display_stderr=True)
    return next_bag_filename
