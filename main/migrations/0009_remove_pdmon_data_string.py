# Generated by Django 3.2.3 on 2021-07-10 09:03

from django.db import migrations


class Migration(migrations.Migration):

    dependencies = [
        ('main', '0008_alter_pdmon_channel_string'),
    ]

    operations = [
        migrations.RemoveField(
            model_name='pdmon',
            name='data_string',
        ),
    ]
