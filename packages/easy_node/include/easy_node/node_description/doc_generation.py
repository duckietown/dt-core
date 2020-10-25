import os

import duckietown_utils as dtu
from .configuration import EasyNodeConfig, load_configuration_for_nodes_in_package, PROCESS_THREADED


def generate_easy_node_docs():
    skip = ["easier_node"]
    packages = dtu.get_list_of_packages_in_catkin_ws()
    dtu.logger.info("Looking in %d packages for nodes." % len(packages))

    names = sorted(packages, key=lambda _: packages[_])
    for package_name in names:
        md = ""
        package_dir = packages[package_name]
        package_xml = os.path.join(package_dir, "package.xml")
        package_info = dtu.read_package_xml_info(package_xml)
        md += generate_from_package_info(package_info, package_dir)

        configs = load_configuration_for_nodes_in_package(package_name)
        for node_name, config in list(configs.items()):
            if node_name in skip:
                continue
            md += generate_from_node_config(config)

        out = os.path.join(package_dir, ".autogenerated.md")
        write_to_file_if_changed(out, md)


branch = "master18"  # FIXME: should not be hard coded
S = "\n\n"


def generate_from_package_info(info, package_dir):
    assert isinstance(info, dtu.PackageXML)
    md = ""
    md += "<div id='%s-autogenerated' markdown='1'>\n\n" % (info.name)
    md += "\n<!-- do not edit this file, autogenerated -->\n\n"

    md += "## Package information " + S

    parent = os.path.basename(os.path.dirname(package_dir))
    unique = "%s/%s" % (parent, info.name)

    md += (
        "[Link to package on Github](github:org=duckietown,repo=Software,path=%s,branch=%s)"
        % (unique, branch)
        + S
    )

    md += '### Essentials {nonumber="1"}' + S

    def is_in(P, where):
        return any(P.email == _.email for _ in where)

    for p in info.authors:

        if is_in(p, info.maintainers):
            md += "Author: " + format_person(p) + " (maintainer)" + S
        else:
            md += "Author: " + format_person(p) + S

    # if not info.authors:
    #     md += 'TODO: add authors.' + S

    for p in info.maintainers:

        if is_in(p, info.authors):
            continue
        else:
            md += "Maintainer: " + format_person(p) + S

    # if not info.maintainers:
    #     md += 'TODO: add maintainers.' + S

    # md += 'TODO: add code to generate list of dependency.' + S

    # md += 'TODO: add code to generate a link to a Github issue table for this package.' + S

    # md += 'TODO: add code to show unit tests build indicator.' + S

    md += '### Description {nonumber="1"}' + S

    if info.description:
        md += info.description + S
    else:
        md += "TODO: no package description found."

    md += S + "</div>" + S

    return md


def format_person(p):
    assert isinstance(p, dtu.Person)
    if p.email:
        return "[%s](mailto:%s)" % (p.name, p.email)
    else:
        return p.name


def generate_from_node_config(config):
    md = ""
    md += "<!-- file start -->\n\n"
    md += "<div id='%s-%s-autogenerated' markdown='1'>\n\n" % (config.package_name, config.node_type_name)
    md += "\n<!-- do not edit this file, autogenerated -->\n\n"

    assert isinstance(config, EasyNodeConfig)
    short = os.path.basename(config.filename)
    md += "(Generated from [configuration `%s`]" % short
    md += "(github:org=duckietown,repo=Software,path=%s,branch=%s).)" % (short, branch) + S

    if config.description is None:
        md += "TODO: Missing node description in `%s`.\n\n" % os.path.basename(config.filename)
    else:
        md += config.description + "\n\n"
    md += generate_configuration(config)
    md += "\n\n</div>"
    return md


def write_to_file_if_changed(filename, contents):
    if os.path.exists(filename):
        existing = open(filename).read()
        need_write = existing != contents
    else:
        need_write = True

    if need_write:
        with open(filename, "w") as f:
            f.write(contents)
        dtu.logger.info("Written to %s" % filename)
    else:
        dtu.logger.info("File already up to date %s" % filename)


def write_desc(x):
    return 'TODO: Missing description for entry "`%s`".' % x.name


# @contract(config=EasyNodeConfig)


def generate_configuration(config):
    assert isinstance(config, EasyNodeConfig)
    md = ""
    COMMON_PREFIX = "en_"

    choose = [_ for _ in list(config.parameters.values()) if not _.name.startswith(COMMON_PREFIX)]

    md += '### Parameters {nonumber="1"}' + S

    if not choose:
        md += "No parameters defined.\n\n"

    for param in choose:
        md += "**Parameter `%s`**: " % param.name
        md += describe_type(param.type)
        if param.has_default:
            md += "; default value: `%r`" % param.default
        md += "\n\n"
        if param.desc:
            md += param.desc
        else:
            md += write_desc(param)

        md += "\n\n"

    md += '### Subscriptions {nonumber="1"}' + S

    if not config.subscriptions:
        md += "No subscriptions defined.\n\n"

    for subscription in list(config.subscriptions.values()):
        md += "**Subscription `%s`**: " % subscription.name
        md += "topic `%s` (%s)\n\n" % (subscription.topic, describe_type(subscription.type))

        if subscription.desc:
            md += subscription.desc
        else:
            md += write_desc(subscription)

        md += "\n\n"

        if subscription.process == PROCESS_THREADED:
            md += "Note: The data is processed *asynchronously* in a different thread.\n\n"

    md += '### Publishers {nonumber="1"}' + S

    if not config.publishers:
        md += "No publishers defined.\n\n"

    for publisher in list(config.publishers.values()):
        md += "**Publisher `%s`**: " % publisher.name
        md += "topic `%s` (%s)\n\n" % (publisher.topic, describe_type(publisher.type))
        if publisher.desc:
            md += publisher.desc
        else:
            md += write_desc(publisher)

        md += "\n\n"

    #     md += '### Services \n\n'
    #
    #
    #     md += '(EasyNode does not parse services.\n\n')

    return md


def describe_type(x):
    if x is None:
        return "not known"
    else:
        return "`%s`" % x.__name__
