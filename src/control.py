#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""A simple cmd2 application."""
import sys, os, glob
import re
import importlib
from multiprocessing import Process
from multiprocessing.queues import Queue
from cmd2 import Cmd, with_argparser
import argparse
from threading import Thread
import shutil
import fileinput
from simulator import start_simulator, create_cmd_parser
from interaction_handler import convert_planner_name_by_uppercase, convert_planner_name_by_underscore
from local_planners.local_planner import LocalPlanner
from global_planners.global_planner import GlobalPlanner
from multiagent_global_planners.multiagent_planner import MultiAgentPlanner
from interaction_handler import convert_to_protocol_data, WARNING_FLAG, SUBPROCESS_END_FLAG, CLICKED_FLAG, RECEIVED_FLAG, CONTROL_FLAG, GET_FLAG

TERMINATE_FLAG = "TERMINATE"
LOG_FLAG = "LOG"
SILENT_FLAG = "SILENT"

class TalkInterface(object):
    def __init__(self, child, out_q):
        self.process = child
        self.out_q = out_q

class RedirectQueue(Queue):
    def __init__(self):
        super(RedirectQueue, self).__init__()
    def write(self, line):
        line = line.strip()
        if len(line)==0: return
        self.put((os.getpid(), line))
    def flush(self):
        pass

def create_control_parser():
    control_parser = argparse.ArgumentParser()
    control_parser.add_argument(
        '--title', action='store', type=str, dest='title', metavar='simulator_title', help='Specify the name of the simulator that the command will go to; if not given, the current active simulator will be the recevier')
    control_parser.add_argument(
        '-a', '--agent', action='store', type=int, nargs='*', dest='ids', metavar='agent_id', help='Specify the agent(s) who shall process the command; if not given, all the agents in current active simulator will receive the command')
    control_parser.add_argument(
        '-p', '--path', action='store', type=str, dest='path', metavar='xy-tuple,xy-tuple',
            help='Specify the start position, waypoints, and destination position in the form of a sequence of (x,y) tuples')
    control_parser.add_argument(
        '--gp', action='store', type=str, dest='global_planner', help='Set the global planner')
    control_parser.add_argument(
        '--lp', action='store', type=str, dest='local_planner', help='Set the local planner')
    return control_parser

def create_template_parser():
    template_parser = argparse.ArgumentParser()
    template_parser.add_argument(
        '--magp', '--multiagent_global_planner', action='store', type=str, dest='magp', metavar='CLASSNAME',
            help='Give a ClassName and create a new Multiagent Global Planner file from template; ClassName must be in CamelCase')
    template_parser.add_argument(
        '--gp', '--global_planner', action='store', type=str, dest='gp', metavar='CLASSNAME',
        help='Give a ClassName and create a new (Single-Agent) Global Planner file from template; ClassName must be in CamelCase')
    template_parser.add_argument(
        '--lp', '--local_planner', action='store', type=str, dest='lp', metavar='CLASSNAME',
        help='Give a ClassName and create a new Local Planner file from template; ClassName must be in CamelCase')
    return template_parser
    

def create_get_parser():
    get_parser = argparse.ArgumentParser()
    get_parser.add_argument(
        '--title', action='store', type=str, dest='title', metavar='simulator_title', help='Specify the name of the simulator that the command will go to; if not given, the current active simulator will be the recevier')
    get_parser.add_argument(
        '-a', '--agent', action='store', type=str, nargs=2, dest='require_agent', metavar=('agent_id', 'variable_to_store_result'), help='Specify the agent who shall return the instance of itself')
    get_parser.add_argument(
        '-s', '--sensor', action='store', type=str, nargs=2, dest='require_sensor', metavar=('sensor_id', 'variable_to_store_result'),
            help='Specify the agent id who shall return the instance of its sensor')
    return get_parser
    
class ControlApp(Cmd):
    def __init__(self):
        self.debug = True

        self.children_process_dict = {}
        self.in_q = RedirectQueue()
        self.active_process_title = None
        self.variable_dict = {}

        Cmd.__init__(self, use_ipython=False)
        self.listener = Thread(target=self.listener_thread, args=(self.in_q, ))
        self.listener.start()

    new_parser = create_cmd_parser(prog='new')
    control_parser = create_control_parser()
    template_parser = create_template_parser()
    get_parser = create_get_parser()

    def listener_thread(self, in_q):
        def retrieve_title_from_pid(pid):
            for title, p_interface in self.children_process_dict.items():
                if pid == p_interface.process.pid:
                    return title
            return None
        log_process = None
        log_title = None
        while True:
            try:
                received = in_q.get(True)
                if received == TERMINATE_FLAG:
                    break # terminate thread
                pid, msg = received
                if pid == GET_FLAG: # returned by get query, store into variable
                    var_name, type_str, obj_str = msg
                    self.variable_dict[var_name] = convert_to_protocol_data(type_str, obj_str)
                    print self.variable_dict[var_name]
                    continue
                if msg.find(CLICKED_FLAG) >= 0:
                    print msg
                elif msg.find(WARNING_FLAG) >= 0:
                    title = retrieve_title_from_pid(pid)
                    print "{}: {}".format(title, msg)
                elif msg.find(RECEIVED_FLAG) >= 0:
                    title = retrieve_title_from_pid(pid)
                    print "{}: {}".format(title, msg)
                elif msg == SUBPROCESS_END_FLAG: # terminate subprocess
                    title = retrieve_title_from_pid(pid)
                    p = self.children_process_dict[title]
                    p.process.join()
                    del self.children_process_dict[title]
                    print "Close simulator:", title
                    # deal with active process
                    if title == self.active_process_title and len(self.children_process_dict) > 0:
                        self.active_process_title = self.children_process_dict.keys()[0]
                        print "Current active simulator:", self.active_process_title
                    else:
                        self.active_process_title = None
                        print "No simulator exists"
                    # deal with log process
                    if log_process == pid:
                        log_process = None
                        log_title = None
                elif msg == LOG_FLAG:
                    log_process = pid # the subprocess's output will be printed
                    log_title = retrieve_title_from_pid(pid)
                elif msg == SILENT_FLAG:
                    log_process = None
                    log_title = None
                else:
                    if pid == log_process:
                        print "{}: {}".format(log_title, msg)
                    continue
            except Exception as e:
                print "Thread process incoming msg failed:", e

    @with_argparser(new_parser)
    def do_new(self, args):
        '''Create a new subprocess to run simulator with a given title'''
        title = args.title
        counter = 0
        while title in self.children_process_dict: # deal with duplicate
            counter += 1
            title = args.title + str(counter)
        args.title = title

        out_q = Queue()
        child_p = Process(target=start_simulator, args=(args, out_q, self.in_q))
        self.children_process_dict[title] = TalkInterface(child_p, out_q)
        child_p.start()
        self.active_process_title = title
        print "Current active simulator:", self.active_process_title

    def complete_new(self, text, line, begidx, endidx):
        flag_dict = \
            {
                '--title': self.children_process_dict.keys(),
                '--gp': [subclass.__name__ for subclass in MultiAgentPlanner.__subclasses__()] + 
                [subclass.__name__ for subclass in GlobalPlanner.__subclasses__()],
                '--lp': [subclass.__name__ for subclass in LocalPlanner.__subclasses__()],
                '--import': [map_name[4:] for map_name in glob.glob('map/*')] # [4:] remove 'map/'
            }
        return self.flag_based_complete(text, line, begidx, endidx, flag_dict=flag_dict)


    def do_quit(self, args):
        """Terminate control panel as well as all simulators"""
        for interface in self.children_process_dict.values():
            interface.process.terminate()
        # terminate thread
        self.in_q.put(TERMINATE_FLAG)
        self.listener.join()
        return True
    
    @with_argparser(control_parser)
    def do_control(self, args):
        """Select a simulator by its title and control the path of one of its agent by agent id"""
        # pass to corresponding simulator
        args.title = self.__process_title(args.title)
        if args.title: # found
            interface = self.children_process_dict[args.title]
            interface.out_q.put((CONTROL_FLAG, args))

    def complete_control(self, text, line, begidx, endidx):
        flag_dict = \
            {
                '--title': self.children_process_dict.keys(),
                '--gp': [subclass.__name__ for subclass in MultiAgentPlanner.__subclasses__()] + 
                [subclass.__name__ for subclass in GlobalPlanner.__subclasses__()],
                '--lp': [subclass.__name__ for subclass in LocalPlanner.__subclasses__()]
            }
        return self.flag_based_complete(text, line, begidx, endidx, flag_dict=flag_dict)


    def do_activate(self, title):
        '''usage: activate TITLE\n\nSet a simulator to be active by its title.'''
        title = self.__process_title(title)
        if title: # found
            self.active_process_title = title
            print "Current active simulator:", self.active_process_title

    def complete_activate(self, text, line, begidx, endidx):
        index_dict = \
            {
                1: self.children_process_dict.keys()  # Tab-complete food items at index 1 in command line
            }
        return self.index_based_complete(text, line, begidx, endidx, index_dict=index_dict)

    def do_log(self, title):
        '''usage: log TITLE\n\nPrint the log of simulator with given title.'''
        title = self.__process_title(title)
        if title: # found
            for t, p_interface in self.children_process_dict.items():
                if t == title:
                    self.in_q.put((p_interface.process.pid, LOG_FLAG))
                    return

    def complete_log(self, text, line, begidx, endidx):
        index_dict = \
            {
                1: self.children_process_dict.keys()  # Tab-complete food items at index 1 in command line
            }
        return self.index_based_complete(text, line, begidx, endidx, index_dict=index_dict)

    def do_silent(self, args):
        '''usage: s/silent\n\nStop logging.'''
        self.in_q.put((-1, SILENT_FLAG))
    
    @with_argparser(template_parser)
    def do_template(self, args):
        '''Create a new planner file from template, the class name must be in CamelCase, e.g. MyOwnAlgorithm'''
        def create_classfile(src_file, dest_file, TEMPLATE_NAME):
            if os.path.exists(dest_file):
                print "Unable to create file {}: file already exist".format(dest_file)
                return
            shutil.copy2(src_file, dest_file)
            # replace the name of TemplateClass to a user specified one: https://stackoverflow.com/a/290494
            for line in fileinput.input(dest_file, inplace=True):
                if line.find(TEMPLATE_NAME) >= 0:
                    print line.replace(TEMPLATE_NAME, class_name), # comma suppress \n
                else:
                    print line,
            fileinput.close()
            temp = dest_file.split('/')
            pkg = '.'.join(temp[1:-1])
            module_name = temp[-1].split('.py')[0]
            importlib.import_module('.'+module_name, package=pkg)
            print "New file created at", dest_file
            try:
                if os.system("code "+dest_file) != 0:
                    raise Exception('')
                else:return
            except:pass
            try:
                if os.system("sublime "+dest_file) != 0:
                    raise Exception('')
                else:return
            except:pass
            try:
                if os.system("open "+dest_file) != 0:
                    raise Exception('')
                else:return
            except:pass
            try:
                if os.system("emacs "+dest_file) != 0:
                    raise Exception('')
                else:return
            except:pass
            try:
                if os.system("vim "+dest_file) != 0:
                    raise Exception('')
                else:return
            except:pass

        def duplication_check(class_name):
            if class_name in LocalPlanner.__subclasses__() or \
                class_name in GlobalPlanner.__subclasses__() or \
                    class_name in MultiAgentPlanner.__subclasses__():
                    print "Unable to create class {}: another class with same name already exist".format(class_name)
                    return True
            return False

        src_file = None
        dest_file = None
        if args.magp:
            if duplication_check(args.magp): return
            module_name, class_name = convert_planner_name_by_uppercase(args.magp)
            src_file = './template/template_multiagent_planner.py'
            dest_file = './multiagent_global_planners/user/'+module_name+'.py'
            create_classfile(src_file, dest_file, 'TemplateMultiagent')
        if args.gp:
            if duplication_check(args.gp): return
            module_name, class_name = convert_planner_name_by_uppercase(args.gp)
            src_file = './template/template_global_planner.py'
            dest_file = './global_planners/user/'+module_name+'.py'
            create_classfile(src_file, dest_file, 'TemplateGlobal')
        if args.lp:
            if duplication_check(args.lp): return
            module_name, class_name = convert_planner_name_by_uppercase(args.lp)
            src_file = './template/template_local_planner.py'
            dest_file = './local_planners/user/'+module_name+'.py'
            create_classfile(src_file, dest_file, 'TemplateLocal')
        

    def do_rm(self, args):
        '''usage: rm CLASSNAME\n\nRemove user created planner'''
        module_name, _ = convert_planner_name_by_uppercase(args)
        module_paths = ['local_planners/', 'global_planners/', 'multiagent_global_planners/']
        err = ''
        for path in module_paths:
            temp_path = path+'user/'+module_name+'.py'
            try:
                os.remove(temp_path)
                os.remove(temp_path+'c') # also remove the binary
                print "Remove file: '{}'".format(temp_path)
                return
            except Exception as e:
                err += str(e)+'\n'
        print err[:-1] # suppress last \n

    def complete_rm(self, text, line, begidx, endidx):
        user_classes = []
        pattern = "*planner*"
        paths = [f for f in glob.glob(pattern)]
        while len(paths) != 0:
            path = paths.pop()
            path += '/user/*planner.py'
            for filepath in glob.glob(path):
                module_name = filepath.split('/')[-1].replace('.py','') # extract module name
                module_name, class_name = convert_planner_name_by_underscore(module_name)
                user_classes.append(class_name.replace('Planner',''))

        index_dict = \
            {
                1: user_classes  # Tab-complete food items at index 1 in command line
            }
        return self.index_based_complete(text, line, begidx, endidx, index_dict=index_dict)

    def do_close(self, title):
        '''usage: close TITLE\n\nClose a simulator with given title'''
        title = self.__process_title(title)
        if title: # found
            self.children_process_dict[title].out_q.put((SUBPROCESS_END_FLAG,''))
            return

    def complete_close(self, text, line, begidx, endidx):
        index_dict = \
            {
                1: self.children_process_dict.keys()  # Tab-complete food items at index 1 in command line
            }
        return self.index_based_complete(text, line, begidx, endidx, index_dict=index_dict)

    @with_argparser(get_parser)
    def do_get(self, args):
        args.title = self.__process_title(args.title)
        if args.title:
            self.children_process_dict[args.title].out_q.put((GET_FLAG, args))

    def complete_get(self, text, line, begidx, endidx):
        flag_dict = \
            {
                '--title': self.children_process_dict.keys(),
            }
        return self.flag_based_complete(text, line, begidx, endidx, flag_dict=flag_dict)
    
    def do_print(self, args):
        '''usage: print VARIABLE_NAME\n\nPrint the content of a variable'''
        argv = args.split('.')
        output = None
        attr_acc_str = ''
        if len(argv) > 0 and argv[0] in self.variable_dict:
            output = self.variable_dict[argv[0]]
            attr_acc_str = argv.pop(0)
        else:
            print "Cannot find variable {}".format(args)
            return
        while len(argv) > 0:
            try:
                current_attr = argv.pop(0)
                output = getattr(output, current_attr)
                attr_acc_str += '.'+current_attr
            except Exception as e:
                print "{} does not have attribute {}".format(attr_acc_str, current_attr)
                return
        print output

    def complete_print(self, text, line, begidx, endidx):
        index_dict = \
            {
                1: self.variable_dict.keys(),  # Tab-complete food items at index 1 in command line
            }
        return self.index_based_complete(text, line, begidx, endidx, index_dict=index_dict)

    def __process_title(self, title):
        if title: # not None
            title = title.replace('"', '') # process string includes whitespace
            if title in self.children_process_dict:
                return title
            else: 
                title = None # cannot find
        if not title: # None-input
            if self.active_process_title: # at leat 1 simulator exist
                return self.active_process_title
        print "Cannot find simulator titled '{}'".format(title)
            

    do_eof = do_quit
    do_q = do_quit
    do_s = do_silent

if __name__ == '__main__':
    # import all planners for tab-completion
    pattern = "*planner*"
    paths = [f for f in glob.glob(pattern)]
    while len(paths) != 0:
        path = paths.pop()
        if path.find('planner.py') >= 0: # a planner
            temp = path.split('.py')[0]
            temp = temp.split('/')
            pkg = '.'.join(temp[:-1])
            module_name = temp[-1]
            try:
                importlib.import_module('.'+module_name, package=pkg)
            except:
                print "Unable to import", module_name
        subpath = path+'/*'
        paths.extend([f for f in glob.glob(subpath)])

    # cmd loop
    c = ControlApp()
    sys.exit(c.cmdloop())