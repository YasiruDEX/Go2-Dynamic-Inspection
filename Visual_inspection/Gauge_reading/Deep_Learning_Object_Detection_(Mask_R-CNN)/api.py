import os
import shutil

from flask import Flask,render_template
from flask_restful import Api
from flask_wtf import FlaskForm

from werkzeug.utils import secure_filename

from wtforms import FileField, SubmitField
from wtforms.validators import InputRequired

from src.read_gauge_meter import predict_gauge_meter


app = Flask(__name__)
app.config['SECRET_KEY'] = 'supersecretkey'
app.config['UPLOAD_FOLDER'] = 'static/files'


class UploadFileForm(FlaskForm):
    file   = FileField("File", validators=[InputRequired()])
    submit = SubmitField("Upload File")

@app.route('/', methods=['GET',"POST"])
def home():
    form = UploadFileForm()
    if form.validate_on_submit():
        
        file      = form.file.data # First grab the file
        file_save = os.path.join(os.path.abspath(os.path.dirname(__file__)),app.config['UPLOAD_FOLDER'])
    
        try: shutil.rmtree(file_save)
        except FileNotFoundError: pass
        
        os.makedirs(file_save, exist_ok=True)

        print(file_save)
        file.save(os.path.join(os.path.abspath(os.path.dirname(__file__)),app.config['UPLOAD_FOLDER'],secure_filename(file.filename))) # Then save the file
        img_path = os.path.join(os.path.abspath(os.path.dirname(__file__)),app.config['UPLOAD_FOLDER'],secure_filename(file.filename))
        
        # load pre-loaded model from global variable
        with app.app_context():
            predict_pointer = predict_gauge_meter('weights/finetunned_realdata_11.pth', img_path)

        return render_template('output.html', value=predict_pointer)
        
    return render_template('index.html', form=form)


if __name__ == '__main__':
    app.run("0.0.0.0", debug=True, port=5000)

    