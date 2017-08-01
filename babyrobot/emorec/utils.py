import csv
import io
import rospy
import svmutil

from babyrobot.emorec import config as emorec_config
from babyrobot.lib import utils as br_utils


def load_libsvm_model(model_file, scale_file=None, classes_file=None):
    model = svmutil.svm_load_model(model_file)
    classes = (parse_classes(classes_file)
               if classes_file is not None else None)
    scale = (parse_libsvm_scale(scale_file)
             if scale_file is not None else None)
    return model, scale, classes


def parse_classes(classes_file):
    with open(classes_file) as fd:
        classes_lines = [l.strip().split(':') for l in fd.readlines()]
    classes = {c[0]: c[1] for c in classes_lines}
    return classes


def parse_libsvm_scale(scale_file):
    with open(scale_file) as fd:
        scale_lines = [l.strip().split(' ') for l in fd.readlines()][2:]
    scale = {s[0]: (float(s[1]), float(s[2])) for s in scale_lines}
    return scale


def parse_weka_output_csv(csv_out):
    no_junk_lines = '\n'.join(csv_out.split('\n')[2:])
    reader = csv.DictReader(io.StringIO(unicode(no_junk_lines)))
    csv_out_dict = reader.next()
    return csv_out_dict


def classify_weka(model_config, arff_features, classes=None):
    with open(emorec_config.WEKA.TEMP_ARFF_FILE, 'w') as fd:
        fd.write(arff_features)
    cmd = ("java -Xmx2048m -cp {0} {1} -l {2} -T {3} -classifications"
           " weka.classifiers.evaluation.output.prediction.CSV"
           .format(emorec_config.WEKA.CLASSPATH,
                   model_config['classifier'],
                   model_config['path'],
                   emorec_config.WEKA.TEMP_ARFF_FILE))
    return_code, stdout, stderr = br_utils.run_cmd(cmd)
    if return_code < 0:
        rospy.logerr('Failed to classify using Weka: {}'.format(cmd))
        rospy.logger(stderr)
        return None
    rospy.logdebug(cmd)
    rospy.logdebug(stdout)
    csv_dict = parse_weka_output_csv(stdout)
    confidence = csv_dict['prediction']
    label = csv_dict['predicted'].split(':')[0]
    emotion = classes[label] if classes is not None else label
    return emotion, confidence


def toLibsvmFeatures(features, scale=None):
    libsvm_features = {str(i + 1): f.feature_value
                       for i, f in enumerate(features)}
    if scale is not None:

        def normalize(x, xmin, xmax):
            return 2.0 * (x - xmin) / (xmax - xmin) - 1.0
        libsvm_features = [{
            int(k): (normalize(v, scale[k][0], scale[k][1])
                     if k in scale else v)
            for k, v in libsvm_features.iteritems()
        }]
    return libsvm_features


def classify_libsvm(model, features, scale=None, classes=None):
    libsvm_feat = toLibsvmFeatures(features, scale=scale)
    svm_predict = br_utils.suppress_print(svmutil.svm_predict)
    if model.get_svm_type() in [3, 4]:
        labels, _, confidences = svm_predict([-1], libsvm_feat, model)
        prediction = str(labels[0])
        confidence = confidences[0][0]
    else:
        labels, _, confidences = svm_predict(
            [-1], libsvm_feat, model, options='-b 1')
        label = int(labels[0])
        confidence = confidences[0][label]
        prediction = classes[str(label)] if classes is not None else str(label)
    return prediction, confidence
