# Generated by Django 3.2.3 on 2021-07-01 10:08

from django.db import migrations, models


class Migration(migrations.Migration):

    dependencies = [
        ('main', '0001_initial'),
    ]

    operations = [
        migrations.RemoveField(
            model_name='pdmon',
            name='value',
        ),
        migrations.AddField(
            model_name='pdmon',
            name='value_string',
            field=models.CharField(blank=True, max_length=100),
        ),
    ]
