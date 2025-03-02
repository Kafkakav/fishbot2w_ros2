from flask import Flask, render_template, redirect, url_for, jsonify

app = Flask(__name__, static_folder='public')

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/logout')
def logout():
    # session.pop('username', None)
    return redirect(url_for('index'))

@app.errorhandler(404)
def page_not_found(e):
    # return render_template('404.html'), 404
    return jsonify({'error': 'File Not Found'}), 404

if __name__ == '__main__':
    app.run(host='0.0.0.0', debug=True)
